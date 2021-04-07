import mysql.connector

mydb = mysql.connector.connect(
  #user = 'root',
  #passwd = 'Efmmp_2019!',
  #host = 'localhost',
  #database = 'nodes',
  user      = 'wladimir',
  passwd    = 'Wladimir123-',
  host      = '190.121.23.218',
  database  = 'filtro',
  )

print(mydb)

my_cursor = mydb.cursor()
post_CMD  = "INSERT INTO test (id,time,pm25,tn,hn,ts,hs) VALUES (%s,%s,%s,%s,%s,%s,%s)"
test1     = ("CIEP-00",1,20.0,25.5,55.5,25.5,55.5)
my_cursor.execute(post_CMD,test1)
mydb.commit()