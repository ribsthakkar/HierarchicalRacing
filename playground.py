import sys
from io import StringIO

oldstdin = sys.stdin
sys.stdin = StringIO('Test')

print(input('testing,'))