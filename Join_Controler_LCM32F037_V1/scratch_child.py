import subprocess
cmd = 'wmic process where (ParentProcessId=2056) get Caption,ProcessId'
res = subprocess.run(cmd, shell=True, capture_output=True, text=True)
print(res.stdout)
