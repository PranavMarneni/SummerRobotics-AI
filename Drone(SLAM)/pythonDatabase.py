import mysql.connector
from mysql.connectot import errorcode

db_connection = mysql.connector.connect(
    host = "localhost",
    user = "root",
    password = "Pranav0317",
    database = "company"
)
