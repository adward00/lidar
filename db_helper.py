# db_helper.py
import pymysql
import json
from datetime import datetime

DB_CONFIG = {
    'host': 'localhost',      
    'user': 'root',           
    'password': '0000',   
    'database': 'lidar_db',
    'charset': 'utf8mb4'
}

def get_connection():
    return pymysql.connect(**DB_CONFIG)

def insert_lidar_data(ranges, action):
    """
    lidar 데이터를 MySQL에 저장
    ranges : list or np.array
    action : str
    """
    conn = get_connection()
    cursor = conn.cursor()
    sql = """
        INSERT INTO lidardata (ranges, `when`, action)
        VALUES (%s, %s, %s)
    """
    ranges_json = json.dumps(ranges.tolist() if hasattr(ranges, "tolist") else ranges)
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    cursor.execute(sql, (ranges_json, timestamp, action))
    conn.commit()
    cursor.close()
    conn.close()
