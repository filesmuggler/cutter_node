#!/usr/bin/env python
"""
Removes NaNs for pcd to mesh conversion
1.0 is interpreted as infinity therefore 
does not exist in the obj mesh
"""
__author__ = 'Krzysztof Stezala <krzysztof.stezala at student.put.poznan.pl>'
__version__ = '0.1'
__license__ = 'MIT'

import sys,getopt
from tqdm import tqdm

def main(argv):
    inputfile = ''
    outputfile = ''
    try:
        opts, args = getopt.getopt(argv, "hi:o:",["ifile=","ofile="])
    except getopt.GetoptError:
        print 'test.py -i <inputfile> -o <outputfile>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'test.py -i <inputfile> -o <outputfile>'
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg
    print 'Input file is "', inputfile
    print 'Output file is "', outputfile
    file1 = open(inputfile,'r')
    lines = file1.readlines()
    newlines = []

    file2 = open(outputfile, 'w')

    for line in tqdm(lines):
        if line.find("nan"):
            newlines.append(line)
        else:
            newlines.append("1.0 1.0 1.0\n")
    file2.writelines(newlines)
    file2.close()
    print("finished")

if __name__ == "__main__":
    main(sys.argv[1:])