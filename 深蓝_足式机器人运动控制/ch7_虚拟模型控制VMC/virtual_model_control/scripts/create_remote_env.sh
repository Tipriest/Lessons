#!/usr/bin/expect
#exit

set ip "192.168.2.1"
set username "ysc"
set passwd "'"

set script_path [file dirname [info script]]
puts ${script_path}

spawn ssh $username@$ip
expect {
    "(yes/no)" {send "yes\r"; exp_continue}
    "password:" {send "$passwd\r"}
}

expect "$username@*"  {send "mkdir /home/$username/rl_deploy \r"}
expect "$username@*"  {send "mkdir /home/$username/rl_deploy/bin \r"}
expect "$username@*"  {send "mkdir /home/$username/rl_deploy/lib \r"}
expect "$username@*"  {send "mkdir /home/$username/rl_deploy/data \r"}
expect "$username@*"  {send "exit\r"}
expect eof 

