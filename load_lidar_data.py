# load_lidar_data.py
import pymysql
import pandas as pd
import json
from db_helper import DB_CONFIG, get_connection

def load_lidar_dataframe():
    # MySQL 연결
    conn = get_connection()
    cursor = conn.cursor(pymysql.cursors.DictCursor)
    
    # 데이터 조회
    sql = "SELECT id, ranges, action, `when` FROM lidardata ORDER BY id ASC"
    cursor.execute(sql)
    rows = cursor.fetchall()
    
    cursor.close()
    conn.close()
    
    # rows -> DataFrame 변환
    data_list = []
    for row in rows:
        # JSON 문자열을 리스트로 변환
        ranges_list = json.loads(row['ranges'])
        # 길이 확인 후 부족하면 360개 맞추기
        if len(ranges_list) != 360:
            ranges_list = (ranges_list + [0.0]*360)[:360]
        # 각 거리값과 action, id, timestamp 합치기
        data_list.append(ranges_list + [row['action']])
    
    # 컬럼 이름 생성
    columns = [f'range_{i}' for i in range(360)] + ['action']
    
    df = pd.DataFrame(data_list, columns=columns)
    return df

if __name__ == "__main__":
    df = load_lidar_dataframe()
    print(df.head())
    print("DataFrame shape:", df.shape)
