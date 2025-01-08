//
// Created by xzhao on 3/20/2022.
//

#include <cmath>
#include "he1n.hpp"
#include "get_prime.h"

#include <boost/multiprecision/cpp_int.hpp>

namespace mp = boost::multiprecision;
namespace dyers {

    PKey keygen(int bit_length, int rho, int rho_) {
        wide_uint_t p = get_prime(bit_length);
        int nu = rho_ - rho;
        wide_uint_t kappa = get_prime(nu);

        PKey key = {kappa, p};
        return key;
    }

    wide_uint_t pgen(int bit_length, int rho_, wide_uint_t p) {
        int eta = std::floor(std::pow(bit_length, 2) / rho_) - bit_length;
        wide_uint_t q = get_prime(eta);
        wide_uint_t modulus = p * q;

        return modulus;
    }

    cipher_text encrypt(int64_t m, PKey key, wide_uint_t modulus) {
        wide_uint_t q = modulus / key.p;
        wide_uint_t r = get_rand(0, q - 1);
        wide_uint_t s = get_rand(0, key.kappa - 1);

        cipher_text res;
        res.setValue((m + s * key.kappa + r * key.p) % modulus);
        res.setMod(modulus);
        return res;
    }

    wide_int_t decrypt(cipher_text c, PKey key) {  // MIGHT NEED `min_residue"
        return static_cast<wide_int_t>(min_residue(c.getValue() % key.p, key.kappa));
        }

    wide_int_t add(wide_int_t c1, wide_int_t c2, wide_uint_t modulus) {
        return (c1 + c2) % modulus;
    }

    wide_int_t mul(wide_int_t c1, wide_int_t c2, wide_uint_t modulus) {
        return (c1 * c2) % modulus;
    }


}