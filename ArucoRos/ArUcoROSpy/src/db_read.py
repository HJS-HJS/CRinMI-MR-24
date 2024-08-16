import sqlite3

dbfile = '/home/kkw0418/Downloads/extractor+matcher/superpoint_max+NN-mutual/outputs/demo/sfm/database.db'

# Create a SQL connection to our SQLite database
con = sqlite3.connect(dbfile)

cur = con.cursor()
i = 0
# The result of a "cursor.execute" can be iterated over by row
for row in cur.execute("SELECT rows FROM keypoints"):
    print(row[0])
    i =+ 1
print(i)

# Be sure to close the connection
con.close()