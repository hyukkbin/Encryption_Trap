// Created by Shane on 4/7/2022.

#ifndef DYERS_NATIVE_HE1N_HPP
#define DYERS_NATIVE_HE1N_HPP

#include <boost/multiprecision/cpp_int.hpp>
#include <boost/multiprecision/cpp_int/serialize.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/multiprecision/cpp_bin_float.hpp>
#include "wide_data.h"

namespace mp = boost::multiprecision;

namespace dyers {

    class cipher_text;

    struct PKey {  // Public Key
        wide_uint_t kappa;
        wide_uint_t p;
    };
    PKey keygen(int bit_length, int rho, int rho_);
    wide_uint_t pgen(int bit_length, int rho_, wide_uint_t p);
    cipher_text encrypt(int64_t m, PKey key, wide_uint_t modulus);
    wide_int_t decrypt(cipher_text c, PKey key);
    wide_int_t add(wide_int_t c1, wide_int_t c2, wide_uint_t modulus);
    wide_int_t mul(wide_int_t c1, wide_int_t c2, wide_uint_t modulus);

    class cipher_text {
    private:

        wide_int_t v;
        wide_uint_t m;

    public:

        std::vector<unsigned char> to_bytearray(){
            std::vector<unsigned char> byte;
            export_bits(v, std::back_inserter(byte),8);

            return byte;
        }

        wide_int_t getValue(void) const {
            return v;
        }
        wide_uint_t getMod(void) const {
            return m;
        }
        void setValue(wide_int_t v) {
            this->v = v;
        }

        void setMod(wide_uint_t mod) {
            this->m = mod;
        }

        /// Set ciphertext value
        cipher_text& operator=(wide_int_t &c1) {
            this->setValue(c1);
            return *this;
        };

        //TODO:move semantics????? std::move???

        /// Set modulus for ciphertext
        cipher_text& operator=(wide_uint_t& mod) {
            this->setMod(m);
            return *this;
        }

        cipher_text operator+(const cipher_text& c2) const {
            cipher_text res;
            res.setValue(dyers::add(this->getValue(), c2.getValue(), m));
            res.setMod(m);
            return res;
        };

        cipher_text * operator+=(const cipher_text& c2) {
            this->setValue(dyers::add(this->getValue(), +c2.getValue(), m));
            return this;
        };

        cipher_text operator-(const cipher_text& c2) const {
            cipher_text res;
            res.setValue(dyers::add(this->getValue(), -c2.getValue(), m));
            res.setMod(m);
            return res;
        };

        cipher_text * operator-=(const cipher_text& c2) {
            this->setValue(dyers::add(this->getValue(), -c2.getValue(), m));
            return this;
        };

        cipher_text operator*(const cipher_text& c2) const {
            cipher_text res;
            res.setValue(dyers::mul(this->getValue(), c2.getValue(), m));
            res.setMod(m);
            return res;

        };

        cipher_text operator - (){
            cipher_text res;

            res.setValue(-this->getValue());
            res.setMod(this->m);
            return res;
        }
        explicit cipher_text (int i = 0) {
            v = 0;
            m = 0;
        }

        std::vector<unsigned char> ctext2arr() {
            std::vector<unsigned char> bytesV;
            mp::export_bits(v,std::back_inserter(bytesV), 8);
            
            // uint8_t* arr = new uint8_t[bytesV.size()];
            // std::memcpy(arr, bytesV.data(),bytesV.size());
            
            return bytesV;
        }

    };


}
#endif //DYERS_NATIVE_HE1N_HPP
