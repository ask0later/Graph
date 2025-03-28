import sys
from subprocess import run, Popen, PIPE
from sys import executable

generator = sys.argv[1]
num_test = 1
is_ok = True
for i in range(1, 13):
    str_data =  "tests/" + str(i) + ".dat"
    file_in = open(str_data, "r")
    str_ans = "tests/" + str(i) + ".ans"

    ans = ""
    for i in open(str_ans):
        ans += i.strip()
	
    result = run([generator], capture_output = True, encoding='cp866', stdin=file_in)
    print("Test: " + str(num_test).strip())

    res = result.stdout.strip()

    is_ok &= (res == ans)
    if res == ans:
        print("OK")
    else:
        print("ERROR\nExpect:", ans, "\nGive:", res)
    print("-------------------------------------------------")
    num_test += 1

if is_ok:
	print("TESTS PASSED")
else:
    print("TESTS FAILED")
    sys.exit(1)