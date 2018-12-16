#!/usr/bin/python
# -*- coding: utf-8 -*-
#sqltest.py - Fetch and display the MySQL database server version.
# import the MySQLdb and sys modules
#deletes first row, runs after the robot finishes order.
import MySQLdb
import sys
import os
import time
# open a database connection
# be sure to change the host IP address, username, password and database name to match your own
connection = MySQLdb.connect(host = "132.178.226.152", user = "test", passwd = "mec202c", db = "lunchbot")
# prepare a cursor object using cursor() method
cursor = connection.cursor ()
# print the row[0]
# (Python starts the first row in an array with the number zero – instead of one)
reddit=0
checker="There is no input"
exists=0
while(exists==0):
	try:
		cursor.execute("SELECT * FROM offices LIMIT 10")
		existential=cursor.fetchone()		
		checkit=existential[0]
		exists=1
	except TypeError as error:
		print "I'm waiting for a command..."		
		time.sleep(5)
		exists=0
		connection.commit() #basically updates the database to realtime
connection.commit()		
while(reddit==0):
        try:
                cursor.execute("SELECT office_id FROM offices WHERE ID='1'")
		row= cursor.fetchone()
                checker=row[0]
		reddit=1
		connection.commit()
        except TypeError as error:
                reddit=0
                cursor.execute("SET @i=0")
		cursor.execute("UPDATE offices SET `ID` = @i:=@i+1;")
                connection.commit()
print "Office ID:", checker
cursor.execute("DELETE FROM offices WHERE ID='1'")
cursor.execute("SET @i=0")
cursor.execute("UPDATE offices SET `ID` = @i:=@i+1;")
connection.commit()
# close the cursor object
cursor.close ()
# close the connection 
connection.close ()
os.system("python go_to_v1.py "+checker)

#NOW WAITS FOR A BUTTON RESPONSE FROM PERSON
#os.system("python go_to_v1.py --"+checker)
#make the program so that if nothing is in queue, it goes home, or else it heads directly to its next destination on queue
# exit the program
#ps I fixed the connection issue by disabling the firewall.
sys.exit()

