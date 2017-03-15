# This is because of Biraj's stupidity
# His concerns are not valid

import os
dir = './'
files = ['ko_akshay.txt', 'n_akshay.txt', 'ni_akshay.txt', 'chi_akshay.txt', 'wa_akshay.txt']

new_strings = []
for name in files:
    final_string = ""
    for line in open(dir + name):
        if line in ["END", "END\n", "END\r\n"]:
            final_string += "END\n"
        else:
            splits = line.split(",")
            new_splits = []
            for i, val in enumerate(splits):
                if i != 6:
                    new_splits.append(val)

            new_line = ",".join(new_splits)
            final_string += new_line

    new_strings.append(final_string)

for string in new_strings:
    print string
    print "Thank you Biraj"
