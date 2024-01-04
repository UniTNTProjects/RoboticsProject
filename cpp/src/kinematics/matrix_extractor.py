# create a python script which save all the matrices in the file "/home/emanuele/Scrivania/Untitled-1.txt" onto a new file formatted as a array of float

import numpy as np
import re

# open the file
f = open("/home/emanuele/Scrivania/Untitled-1.txt", "r")

# read the file
data = f.read()

# close the file
f.close()

# extract the matrices from the file
# the matrices are found in this form:
#  abrotation: 1  0  0
#  0  0  1
#  0 -1  0
# you have to extract the following as [1,0,0,0,0,1,0,-1,0]
# you can find the matrices by searching the string "abrotation: "
matrix = []

# find the string "abrotation: "
abrotation = re.search("abrotation: ", data).end()
# repeat for all the occurences of the string "abrotation: "

couunter = 0


while abrotation != None:
    # extract the matrix

    for i in range(9):
        # find the next number
        number = re.search("-?[0-9]+", data[abrotation:])
        # print(number, end=" ")
        # add the number to the matrix
        matrix.append(
            float(data[abrotation + number.start() : abrotation + number.end()])
        )
        # move abrotation at the end of the number
        abrotation = abrotation + number.end()

    # print the matrix found, only the last 9 elements, and the counter
    print(matrix[-9:])
    print(couunter)
    couunter += 1
    if couunter > 50:
        break

    # find the string "abrotation: "
    try:
        abrotation = re.search("abrotation: ", data[abrotation:]).end() + abrotation
    except:
        abrotation = None


# save the matrix in a file
f = open("/home/emanuele/Scrivania/Untitled-2.txt", "w")
f.write(str(matrix))
f.close()
