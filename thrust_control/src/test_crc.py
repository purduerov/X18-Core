def crc_remainder(input_bitstring, polynomial_bitstring, initial_filler):
    """Calculate the CRC remainder of a string of bits using a chosen polynomial.
    initial_filler should be '1' or '0'.
    """
    polynomial_bitstring = polynomial_bitstring.lstrip('0')
    len_input = len(input_bitstring)
    initial_padding = (len(polynomial_bitstring) - 1) * initial_filler
    input_padded_array = list(input_bitstring + initial_padding)
    while '1' in input_padded_array[:len_input]:
        cur_shift = input_padded_array.index('1')
        for i in range(len(polynomial_bitstring)):
            input_padded_array[cur_shift + i] \
            = str(int(polynomial_bitstring[i] != input_padded_array[cur_shift + i]))
    return ''.join(input_padded_array)[len_input:]

def crc_check(input_bitstring, polynomial_bitstring, check_value):
    """Calculate the CRC check of a string of bits using a chosen polynomial."""
    polynomial_bitstring = polynomial_bitstring.lstrip('0')
    len_input = len(input_bitstring)
    initial_padding = check_value
    input_padded_array = list(input_bitstring + initial_padding)
    while '1' in input_padded_array[:len_input]:
        cur_shift = input_padded_array.index('1')
        for i in range(len(polynomial_bitstring)):
            input_padded_array[cur_shift + i] \
            = str(int(polynomial_bitstring[i] != input_padded_array[cur_shift + i]))
    return ('1' not in ''.join(input_padded_array)[len_input:])

if __name__ == "__main__":
    polynomial = bin(79764919)
    polynomial = polynomial.lstrip('0b')
    print(polynomial)

    message = bin(2859610287596948597295304)
    message = message.lstrip('0b')
    print (message)

    array = crc_remainder(message, polynomial, '0')
    print(array)
    result = crc_check(message, polynomial, array)

    # array = crc_remainder('001001011101100010111100100011001000110010001100100011001000110010001100100011001000', '0000100110000010001110110110111', '0')
    # print(array)
    # result = crc_check('001001011101100010111100100011001000110010001100100011001000110010001100100011001000', '0000100110000010001110110110111', array)
    print(result)

'''

polynomial: CRC-CCITT (CRC-16, thus 17 bit polynomial)
x16 + x12 + x5 + 1
1 0001 0000 0010 0001
11021

message_Type (8 bits) -> 0 to 255
message_ID (16 bits) -> 0 to 65535
data (64 bits) -> 8 * 0 to 255

pad concatenated inputs by 16 bits to account for crc
assume message_type is full thrust control: (3)
assume message_ID is 65519

0000 0011, 1111 1111 1110 1111, 1000 0000 1000 0000 1000 0000 1000 0000 1000 0000 1000 0000 1000 0000 1000 0000, 0000 0000 0000 0000
divide and calculate remainder for 1 0001 0000 0010 0001


def calculate_remainder(message_Type, message_ID, data):

def to_binary_list(int_base_10):
    #convert an int base 10 to a list of 0's and 1's
    return [int(x) for x in bin(int_base_10)[2:]]

def divide(dividend: int, divisor: int) -> int:
        negative = (dividend > 0 and divisor < 0) or (dividend < 0 and divisor > 0)
        
        upper_bound = 1 << 31 if negative else (1 << 31) -1
        
        dividend = 0 - dividend if dividend < 0 else dividend
        divisor = 0 - divisor if divisor < 0 else divisor
        
        dividend = to_binary_list(dividend)
        
        current_dividend = 0
        result = 0
        for next_digit in dividend:
            current_dividend = (current_dividend << 1) + next_digit

            if(divisor <= current_dividend):
                current_dividend -= divisor
                new_digit = 1
            else:
                new_digit = 0
            
            result = (result << 1) + new_digit
        
        result = min(result, upper_bound)
        if(negative):
            result = 0 - result
        
        return result
'''