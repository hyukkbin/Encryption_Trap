//
// Created by xzhao on 3/26/2022.
//

#ifndef DYER_S_GET_PRIME_H
#define DYER_S_GET_PRIME_H

#define FILE_NAME "primes.txt"
#define NO_PRIME 0

#include <string>
#include <iostream>
#include <fstream>
#include "wide_data.h"

wide_uint_t get_prime(int bit_length);
wide_uint_t get_rand(wide_uint_t min, wide_uint_t max);
wide_int_t min_residue(const wide_int_t&  a, const wide_int_t& m);


//uint64_t get_rand_bits(int bit_length);
#endif //DYER_S_GET_PRIME_H
