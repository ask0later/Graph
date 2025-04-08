import sys
from subprocess import run, PIPE

generator = sys.argv[1]
num_test = 1
is_ok = True

for i in range(1, 14):
    test_file = f"tests/{i}.dat"
    ans_file = f"tests/{i}.ans"
    
    with open(test_file, "r") as f_in:
        input_data = f_in.read()
    
    with open(ans_file, "r") as f_ans:
        ans = f_ans.read().replace('\r\n', '\n').rstrip()

    result = run([generator], input=input_data, capture_output=True, text=True, encoding='cp866')
    res = (result.stdout + result.stderr).replace('\r\n', '\n').rstrip()

    print(f"Test {num_test}:")
    
    is_current_ok = (res == ans)
    is_ok &= is_current_ok
    
    if is_current_ok:
        print("OK")
    else:
        print("ERROR")
        print("Expected:\n", repr(ans))
        print("Got:\n", repr(res))
    
    print("-------------------------------------------------")
    num_test += 1

if is_ok:
    print("TESTS PASSED")
else:
    print("TESTS FAILED")
    sys.exit(1)