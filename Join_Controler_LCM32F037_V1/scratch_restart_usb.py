import subprocess
cmd = "powershell -Command \"Get-PnpDevice -FriendlyName '*J-Link*' | ForEach-Object { Disable-PnpDevice -InstanceId $_.InstanceId -Confirm:$false; Start-Sleep -Milliseconds 500; Enable-PnpDevice -InstanceId $_.InstanceId -Confirm:$false }\""
res = subprocess.run(cmd, shell=True, capture_output=True, text=True)
print('STDOUT:', res.stdout)
print('STDERR:', res.stderr)
