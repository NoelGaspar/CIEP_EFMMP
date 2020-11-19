import mysql.connector

mydb = mysql.connector.connect(
  user = 'root',
  passwd = 'Efmmp_2019!',
  host = 'localhost',
  database = 'nodes',
  )

print(mydb)

my_cursor = mydb.cursor()
post_CMD  = "INSERT INTO ciep_test (node_id,time_,count,sht_t,sht_h,dht_t,dht_h,pms_25,pms_t,pms_h) VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)"
test1     = ("em-ciep-00",0,20,25.5,55.5,25.5,55.5,12,25.5,55.5)

post_test = "INSERT INTO test (id_nodo,temp,cknumb) VALUES (%s,%s,%s)"
test_data = ("em-ciep-00",25.5,20)


my_cursor.execute(post_CMD,test1)
#my_cursor.execute(post_test,test_data)
mydb.commit()


