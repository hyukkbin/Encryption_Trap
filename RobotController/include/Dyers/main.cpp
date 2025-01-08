#include <iostream>

#include "he1n.hpp"

#include "wide_data.h"

#include <boost/multiprecision/cpp_int.hpp>

#include "get_prime.h"


//namespace mp = boost::multiprecision;
#define MAX_LAMBDA 4096
// NOTE: If getting error 0xc0000094-integer-division-by-zero, then eta is likely too large and `wide_int_t` cannot hold an int of size 2^eta
int main() {
using namespace dyers;
/*    for (int bit_length = 32; bit_length < MAX_LAMBDA; ++bit_length) {
        std::cout  << "bit_length: " << bit_length << std::endl;
        get_prime(bit_length);
    }
    */
    
    
    int bit_length = 256;
    int rho = 1;
    int rho_ = 32;
    float delta = .01;

    PKey key = keygen(bit_length, rho, rho_);
    std::cout << key.p << std::endl;
    auto modulus = pgen(bit_length, rho_, key.p);

    int m1 = 56;
    int m2 = -2;

/*    auto c1 = enc<wide_double_t>(m1, key, modulus, delta);
    auto c2 = enc<wide_double_t>(m2, key, modulus, delta);*/
    auto c1 = encrypt(m1,key, modulus);
    auto c2 = encrypt(m2,key, modulus);

    auto c3 = c1 + c2;
    auto c4 = c1 * c2;

    auto m1_dec = decrypt(c1, key);
    auto m2_dec = decrypt(c2, key);
    auto m3_dec = decrypt(c3, key);
    auto m4_dec = decrypt(c4, key);

    std::cout << "dec(c1) = " << m1_dec << std::endl;
    std::cout << "dec(c2) = " << m2_dec << std::endl;
    std::cout << "dec(c1 + c2) = " << m3_dec << std::endl;
    std::cout << "dec(c1 * c2) = "<< m4_dec << std::endl;

    return 0;
}