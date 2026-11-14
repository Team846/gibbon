if not exist robot_logs mkdir robot_logs
scp -r "lvuser@10.8.46.2:/home/lvuser/logs/*" robot_logs/
scp -r "lvuser@10.8.46.2:/u/logs/*" robot_logs/ 2>nul
