//
// Created by xzhao on 3/22/2022.
//
//
// Created by xzhao on 3/20/2022.
//

#include <random>
#include "get_prime.h"

#include <boost/multiprecision/cpp_int.hpp>
#include <boost/multiprecision/miller_rabin.hpp>
#include <boost/random.hpp>


namespace mp = boost::multiprecision;
namespace rng = boost::random;

//bool probably_prime(wide_uint_t n) {  // miller_rabin_test is a probabilistic primality test.
//    static rng::mt19937 gen2(clock());
//    return mp::miller_rabin_test<mp::cpp_int>(n, 25, gen2);
//}

wide_int_t min_residue(const wide_int_t&  a, const wide_int_t& m) {
    wide_int_t b = a % m;
    wide_int_t c = b - m;
    return b >= abs(c) ? c:b;
}

// TODO: `get_rand` and `get_rand_bits` could be tuned for performance by using `rng::independent_bit_engine`
wide_uint_t get_rand(const wide_uint_t min, const wide_uint_t max) {
    static rng::mt11213b rng(clock());
    rng::uniform_int_distribution<wide_uint_t> distribution(min,max);

    return distribution(rng);
}
//wide_uint_t get_rand_bits(int bit_length) {
//    static constexpr wide_uint_t base_two = wide_uint_t(2);
//    wide_uint_t start = mp::pow(base_two, bit_length-1);
//    wide_uint_t range = mp::pow(base_two, bit_length);
//    return get_rand(start, range);
//}

//wide_uint_t generate_prime(int bit_length){
//    wide_uint_t p = get_rand_bits(bit_length);
//    while(!probably_prime(p)){
//        p = get_rand_bits(bit_length);
//    }
//
//    return p;
//}


//void write_prime_to_file(int bit_length, const wide_uint_t& prime)
//{
//    std::ofstream myfile;
//    myfile.open(FILE_NAME, std::ios_base::app); // std::ios_base::app ==> append not overwrite
//    myfile << bit_length << "," << prime << std::endl;
//    myfile.close();
//}

wide_uint_t read_prime_from_file(const int bit_length)
{
    std::ifstream myfile;
    myfile.open(FILE_NAME); // std::ios_base::app ==> append not overwrite

    std::string line;

    wide_uint_t found_prime = NO_PRIME;
    while (std::getline(myfile, line)) {
        std::size_t found_at = line.find(',');

        std::string bit_length_str = line.substr(0, found_at);
        int found_bit_length = std::stoi(bit_length_str);

        if (found_bit_length == bit_length) {
            std::string prime_str = line.substr(found_at+1);
            wide_uint_t prime(prime_str);
            found_prime = prime;
            break;
        }
    }

    myfile.close();
    return found_prime;
}


wide_uint_t get_prime(const int bit_length){
    wide_uint_t p = read_prime_from_file(bit_length);
//    if (p == NO_PRIME) {
//        p = generate_prime(bit_length);
//        write_prime_to_file(bit_length, p);
//    }

    return p;
}
