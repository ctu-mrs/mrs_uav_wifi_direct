// SPDX-License-Identifier: BSD-3-Clause
/** @file certificates.hpp
 * @brief Create certificates and remember the certificate presented by each UAV.
 */
#pragma once

#include <fcntl.h>
#include <openssl/ec.h>
#include <openssl/pem.h>
#include <openssl/rand.h>
#include <openssl/x509v3.h>
#include <sys/file.h>

#include <fstream>
#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#include "config.hpp"

namespace mrs_uav_wifi_direct {
/** @brief Protect certificate creation and peer records with a shared file lock. */
class IdentityLock {
	public:
	/** @brief Keep the identity files locked while a process reads or updates them. */
	explicit IdentityLock(const std::string& path) {
		fd_ = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
		if (fd_ < 0 || ::flock(fd_, LOCK_EX) != 0) {
			if (fd_ >= 0) {
				::close(fd_);
			}
			throw std::runtime_error("Cannot lock TLS identity directory: " + path);
		}
	}
	/** @brief Release the lock when the file is closed. */
	~IdentityLock() {
		::close(fd_);
	}
	IdentityLock(const IdentityLock&) = delete;
	IdentityLock& operator=(const IdentityLock&) = delete;

	private:
	int fd_{-1};
};

/**
 * @brief Create a self-signed identity once and remember each peer's first certificate.
 * TLS proves possession of the certificate's private key. Configured names and IPs
 * restrict first contact; the persisted SHA-256 fingerprint authenticates subsequent
 * connections. First contact requires a trusted network. An explicitly supplied CA
 * instead enables MsQuic's normal chain validation and bypasses TOFU pin storage.
 */
class Certificates {
	public:
	/** @brief Store the configured paths; prepare() loads or creates the files. */
	Certificates(const Config& config, rclcpp::Logger logger) : config_(config), logger_(logger) {}

	/**
	 * @brief Load or generate a persistent P-256 key and ten-year self-signed identity.
	 * The lock makes simultaneous node startups safe. A key committed before a crash
	 * is reused when its missing certificate is created. If a certificate's key is
	 * missing, startup stops so the UAV keeps the identity its peers already trust.
	 */
	void prepare() {
		private_directory(config_.identity_directory);
		IdentityLock lock(config_.identity_directory + "/identity.lock");
		const bool has_key = std::filesystem::exists(config_.key);
		const bool has_cert = std::filesystem::exists(config_.cert);
		if (has_cert && !has_key) {
			throw std::runtime_error("TLS certificate exists without its private key: " + config_.key);
		}
		Key key(nullptr, EVP_PKEY_free);
		if (has_key) {
			Bio input(BIO_new_file(config_.key.c_str(), "r"), BIO_free);
			if (input) {
				key.reset(PEM_read_bio_PrivateKey(input.get(), nullptr, nullptr, nullptr));
			}
			check(static_cast<bool>(key), "Reading TLS private key");
		} else {
			check(config_.automatic_identity, "Automatic identity creation is disabled for supplied TLS files");
			std::unique_ptr<EVP_PKEY_CTX, decltype(&EVP_PKEY_CTX_free)> context(
				EVP_PKEY_CTX_new_id(EVP_PKEY_EC, nullptr), EVP_PKEY_CTX_free);
			check(context && EVP_PKEY_keygen_init(context.get()) == 1 &&
					EVP_PKEY_CTX_set_ec_paramgen_curve_nid(context.get(), NID_X9_62_prime256v1) == 1,
				"Initializing TLS key generation");
			EVP_PKEY* generated{};
			check(EVP_PKEY_keygen(context.get(), &generated) == 1, "Generating TLS private key");
			key.reset(generated);
			atomic_pem(config_.key, key.get(), nullptr);
		}
		Cert certificate(nullptr, X509_free);
		if (has_cert) {
			Bio input(BIO_new_file(config_.cert.c_str(), "r"), BIO_free);
			if (input) {
				certificate.reset(PEM_read_bio_X509(input.get(), nullptr, nullptr, nullptr));
			}
			check(static_cast<bool>(certificate), "Reading TLS certificate");
		} else {
			check(config_.automatic_identity, "A configured CA requires an existing CA-signed local identity");
			certificate.reset(X509_new());
			check(static_cast<bool>(certificate), "Allocating TLS certificate");
			uint64_t serial{};
			check(RAND_bytes(reinterpret_cast<unsigned char*>(&serial), sizeof(serial)) == 1,
				"Generating certificate serial");
			check(X509_set_version(certificate.get(), 2) == 1 &&
					ASN1_INTEGER_set_uint64(X509_get_serialNumber(certificate.get()), serial ? serial : 1) == 1 &&
					X509_gmtime_adj(X509_getm_notBefore(certificate.get()), -300) &&
					X509_gmtime_adj(X509_getm_notAfter(certificate.get()), 10L * 365 * 24 * 60 * 60) &&
					X509_set_pubkey(certificate.get(), key.get()) == 1,
				"Setting certificate fields");
			const auto name = config_.name(config_.own);
			auto* subject = X509_get_subject_name(certificate.get());
			check(X509_NAME_add_entry_by_txt(subject, "CN", MBSTRING_ASC,
					  reinterpret_cast<const unsigned char*>(name.c_str()), -1, -1, 0) == 1 &&
					X509_set_issuer_name(certificate.get(), subject) == 1,
				"Setting certificate identity");
			add_extension(certificate.get(), NID_basic_constraints, "critical,CA:FALSE");
			add_extension(certificate.get(), NID_key_usage, "critical,digitalSignature");
			add_extension(certificate.get(), NID_ext_key_usage, "serverAuth,clientAuth");
			// Authenticate the UAV name independently of either network's address.
			// This also permits certificate creation while both interfaces are offline.
			add_extension(certificate.get(), NID_subject_alt_name, "DNS:" + name);
			check(X509_sign(certificate.get(), key.get(), EVP_sha256()) > 0, "Signing TLS certificate");
			atomic_pem(config_.cert, nullptr, certificate.get());
			RCLCPP_INFO(logger_, "Created persistent TLS identity for %s in %s", name.c_str(),
				config_.identity_directory.c_str());
		}
		check(X509_check_private_key(certificate.get(), key.get()) == 1 && valid(certificate.get()) &&
				X509_check_host(certificate.get(), config_.name(config_.own).c_str(), 0, X509_CHECK_FLAG_NO_WILDCARDS,
					nullptr) == 1,
			"Local TLS identity is invalid, expired or belongs to a different UAV");
		if (config_.ca.empty()) {
			private_directory(config_.identity_directory + "/peers");
		}
	}

	/** @brief Check certificate lifetime independently of MsQuic's selected validation mode. */
	static bool valid(X509* certificate) {
		return X509_cmp_current_time(X509_get0_notBefore(certificate)) < 0 &&
			X509_cmp_current_time(X509_get0_notAfter(certificate)) > 0;
	}

	/**
	 * @brief Pin a validated configured peer on first contact, rejecting later changes.
	 * Call only after checking the expected UAV name and remote address. A mismatch
	 * never overwrites a pin. Resetting trust requires deliberately deleting that peer's
	 * pin on the receiving UAV after verifying the replacement identity out of band.
	 */
	bool accept(X509* certificate, int peer) {
		if (!valid(certificate)) {
			return false;
		}
		if (!config_.ca.empty()) {
			return true;  // MsQuic validates the configured CA chain before this callback.
		}
		Key public_key(X509_get_pubkey(certificate), EVP_PKEY_free);
		if (!public_key || X509_verify(certificate, public_key.get()) != 1) {
			return false;
		}
		std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
		unsigned int length{};
		check(X509_digest(certificate, EVP_sha256(), digest.data(), &length) == 1, "Hashing peer certificate");
		std::ostringstream encoded;
		encoded << std::hex << std::setfill('0');
		for (unsigned int i = 0; i < length; ++i) {
			encoded << std::setw(2) << static_cast<unsigned int>(digest[i]);
		}
		const auto path = config_.identity_directory + "/peers/" + config_.name(peer) + ".sha256";
		IdentityLock lock(config_.identity_directory + "/identity.lock");
		if (std::filesystem::exists(path)) {
			std::ifstream input(path);
			std::string known;
			std::getline(input, known);
			if (known == encoded.str()) {
				return true;
			}
			RCLCPP_ERROR(logger_,
				"Certificate for %s changed; connection rejected. Verify the new identity before resetting %s",
				config_.name(peer).c_str(), path.c_str());
			return false;
		}
		// Exclusive creation prevents replacement, even if another process ignores the lock.
		const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
		check(fd >= 0, "Creating peer certificate pin");
		const auto text = encoded.str() + "\n";
		const bool saved =
			::write(fd, text.data(), text.size()) == static_cast<ssize_t>(text.size()) && ::fsync(fd) == 0;
		::close(fd);
		check(saved, "Saving peer certificate pin");
		RCLCPP_INFO(logger_, "Pinned first certificate for %s: %s", config_.name(peer).c_str(), encoded.str().c_str());
		return true;
	}

	private:
	using Key = std::unique_ptr<EVP_PKEY, decltype(&EVP_PKEY_free)>;
	using Cert = std::unique_ptr<X509, decltype(&X509_free)>;
	using Bio = std::unique_ptr<BIO, decltype(&BIO_free)>;

	/** @brief Report certificate and file errors to the caller. */
	static void check(bool success, const char* operation) {
		if (!success) {
			throw std::runtime_error(operation);
		}
	}
	/** @brief Give the identity directory access permissions for its owner. */
	static void private_directory(const std::string& path) {
		std::filesystem::create_directories(path);
		std::filesystem::permissions(path, std::filesystem::perms::owner_all);
	}
	/** @brief Add one standard X.509 v3 extension to a self-signed leaf identity. */
	static void add_extension(X509* certificate, int nid, const std::string& value) {
		X509V3_CTX context{};
		X509V3_set_ctx(&context, certificate, certificate, nullptr, nullptr, 0);
		std::unique_ptr<X509_EXTENSION, decltype(&X509_EXTENSION_free)> extension(
			X509V3_EXT_conf_nid(nullptr, &context, nid, value.c_str()), X509_EXTENSION_free);
		check(extension && X509_add_ext(certificate, extension.get(), -1) == 1, "Adding certificate extension");
	}
	/** @brief Write a PEM file with owner-only access, then rename it into place. */
	static void atomic_pem(const std::string& path, EVP_PKEY* key, X509* certificate) {
		std::string pattern = path + ".XXXXXX";
		const int fd = ::mkstemp(pattern.data());
		check(fd >= 0, "Creating TLS identity temporary file");
		bool saved{};
		{
			Bio output(BIO_new_fd(fd, BIO_NOCLOSE), BIO_free);
			saved = output &&
				(key ? PEM_write_bio_PrivateKey(output.get(), key, nullptr, nullptr, 0, nullptr, nullptr)
					 : PEM_write_bio_X509(output.get(), certificate)) == 1 &&
				BIO_flush(output.get()) == 1 && ::fsync(fd) == 0;
		}
		::close(fd);
		if (!saved || ::rename(pattern.c_str(), path.c_str()) != 0) {
			::unlink(pattern.c_str());
			throw std::runtime_error("Saving TLS identity: " + path);
		}
	}
	const Config& config_;
	rclcpp::Logger logger_;
};
}  // namespace mrs_uav_wifi_direct
