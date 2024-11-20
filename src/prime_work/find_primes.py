'''
This script searches for prime numbers within a given bit size range
'''

from prime_list import get_prime_list

for bits in range(4130,8000):
    get_prime_list(bits)