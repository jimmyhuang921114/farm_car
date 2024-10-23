import math
import json


def main():
    file_path='data.json'
    with open(file_path,'r',encoding='utf-8') as file:
        data=json.load(file)
    v=10
    W=0.5*math.pi
    L=data['wheel_base']
    r=data['wheel_radius']
    
    W_L,W_R=calculate_motor_speed(v,W,L,r)
    

if __name__=='__main__':
    main()