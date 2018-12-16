import paramiko
import time
import os
# ssh 
outside=0
print 'enter ssh'
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy()) # this will automatically add the keys
ssh.connect("tavernaweb", username="kjin", password="lunchbot")

# Run your commands
# example 1 : ls command
print 'sqltest below'
stdin, stdout, stderr = ssh.exec_command('python sqltest.py')
yolo = stdout.readlines()
print yolo[0]
time.sleep(2)
# example 2 : change ip address
print 'changing ip address'
stdin, stdout, stderr = ssh.exec_command('sed -i -- s/'+'132.178.226.152'+'/'+'10.254.38.45'+'/g /etc/sysconfig/network-scripts/ifcfg-eth0')
print stdout.readlines()
time.sleep(2)
os.system("python go_to_v1.py "+yolo[0])
time.sleep(5)
while(outside<4):
	print 'enter ssh'
	ssh = paramiko.SSHClient()
	ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy()) # this will automatically add the keys
	ssh.connect("tavernaweb", username="kjin", password="lunchbot")

	print 'sqlsecond below'
	stdin, stdout, stderr = ssh.exec_command('python sqlsecond.py')
	yolo = stdout.readlines()
	print yolo[0]
	time.sleep(2)
	# example 2 : change ip address
	print 'changing ip address'
	stdin, stdout, stderr = ssh.exec_command('sed -i -- s/'+'132.178.226.152'+'/'+'10.254.38.45'+'/g /etc/sysconfig/network-scripts/ifcfg-eth0')
	print stdout.readlines()
	time.sleep(2)
	os.system("python go_to_v1.py "+yolo[0])
	time.sleep(5)

