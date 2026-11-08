import subprocess

cmd = ['powershell', '-NoProfile', '-Command', "Get-PnpDevice | Where-Object { $_.FriendlyName -match 'J-Link|Segger' } | Format-Table Status, Class, FriendlyName"]
res = subprocess.run(cmd, capture_output=True, text=True)
print(res.stdout)
