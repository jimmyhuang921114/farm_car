import math
import json
def calculate_motor_speed(v,W,L,r):
    left_speed=v-(L/2)*W
    right_speed=v+(L/2)*W
    
    
    left_speed=left_speed
    right_speed=right_speed
    
    
    W_L=left_speed/r
    W_R=right_speed/r
    
    print(f'left_motor:{left_speed:.2f}m/s')
    print(f'right_motor:{right_speed:.2f}m/s')
    return W_L,W_R

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