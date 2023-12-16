# This example shows the usage of intermediate waypoints. It will only work with Ruckig Pro or enabled Online API (e.g. default when installed by pip / PyPI).
 
from copy import copy
from pathlib import Path
from sys import path
import csv
import re
import json
from ruckig import InputParameter, OutputParameter, Result, Ruckig
 

if __name__ == '__main__':
    # Create instances: the Ruckig OTG as well as input and output parameters
    otg = Ruckig(3, 0.01, 4)  # DoFs, control cycle rate, maximum number of intermediate waypoints for memory allocation    
    inp = InputParameter(3)  # DoFs
    out = OutputParameter(3, 4)  # DoFs, maximum number of intermediate waypoints for memory allocation
    mechanisms = ["SHOULDER","EXTEND","WRIST"]
  
    with open("profile.json", 'r') as f:
      profile = json.load(f)
    for i in profile:
      name = i['profile']
      
      arrayLength = len(i['mechanisms']['SHOULDER'])
      currentPos = [
        i['mechanisms']['SHOULDER'][0],
        i['mechanisms']['EXTEND'][0],
        i['mechanisms']['WRIST'][0]
      ]
      
      index = 1
      intermediatePos =[]
      while(index < arrayLength-1):
        intermediatePos.append([
            i['mechanisms']['SHOULDER'][index],
            i['mechanisms']['EXTEND'][index],
            i['mechanisms']['WRIST'][index]
        ])
        index+=1
        
      targetPos = [
        i['mechanisms']['SHOULDER'][arrayLength-1],
        i['mechanisms']['EXTEND'][arrayLength-1],
        i['mechanisms']['WRIST'][arrayLength-1]
      ]
      
      inp.current_position = currentPos
      inp.current_velocity = [0, 0, 0]
      inp.current_acceleration = [0, 0, 0]
  
      inp.intermediate_positions = intermediatePos
  
      inp.target_position = targetPos
      inp.target_velocity = [0, 0, 0]
      inp.target_acceleration = [0, 0, 0]
  
      inp.max_velocity = [200, 60, 300]
      inp.max_acceleration = [150, 100, 1200]
      inp.max_jerk = [500, 300, 6000]

      # Define a minimum duration per section of the trajectory (number waypoints + 1)
      inp.per_section_minimum_duration = [0, 0, 0, 0]
  
  
  
      print('\t'.join(['t'] + ["d"+str(i) for i in range(otg.degrees_of_freedom)]+ ["v"+str(i) for i in range(otg.degrees_of_freedom)]+ ["a"+str(i) for i in range(otg.degrees_of_freedom)]))
  
      # Generate the trajectory within the control loop
      first_output, out_list = None, []
      res = Result.Working
      time0, position0, velocity0, acceleration0 = [], [], [], []
      time1, position1, velocity1, acceleration1 = [], [], [], []
      time2, position2, velocity2, acceleration2 = [], [], [], []
      while res == Result.Working:
          res = otg.update(inp, out)
          d = out.new_position
          v = out.new_velocity
          a = out.new_acceleration
          print('\t'.join([f'{out.time:0.3f}'] + [f'{p:0.3f}' for p in out.new_position]+ [f'{v:0.3f}' for v in out.new_velocity]+ [f'{a:0.3f}' for a in out.new_acceleration]))
          out_list.append(copy(out))
          # 0
          time0.append(f'{out.time:0.3f}')
          position0.append(round(d[0], 3))
          velocity0.append(round(v[0], 3))
          acceleration0.append(round(a[0], 3))
          # 1
          time1.append(f'{out.time:0.3f}')
          position1.append(round(d[1], 3))
          velocity1.append(round(v[1], 3))
          acceleration1.append(round(a[1], 3))
          # 2
          time2.append(f'{out.time:0.3f}')
          position2.append(round(d[2], 3))
          velocity2.append(round(v[2], 3))
          acceleration2.append(round(a[2], 3))
          out.pass_to_input(inp)
  
          if not first_output:
              first_output = copy(out)
              
  
      def write_csv_files(filename, headers, data):
          with open(filename, mode='w', newline='') as csv_file:
              writer = csv.writer(csv_file, delimiter=",", quotechar=' ', quoting=csv.QUOTE_MINIMAL)
              writer.writerow(headers)
              for row in data:
                  writer.writerow(row)
              csv_file.seek(0,2) # move the file pointer to the end of the file
              csv_file.seek(csv_file.tell() - 2, 0) # move the file pointer back 2 bytes
              csv_file.truncate() # remove the last newline character
  
      write_csv_files(mechanisms[0] + "_" + name + '.csv', ['t'] + ['d0']+ ["v0"] + ["a0"],[[t] + [p] + [v] + [a] for t, p, v, a in zip(time0, position0, velocity0, acceleration0)])
      write_csv_files(mechanisms[1] + "_" + name + '.csv', ['t'] + ['d1']+ ["v1"] + ["a1"],[[t] + [p] + [v] + [a] for t, p, v, a in zip(time1, position1, velocity1, acceleration1)])
      write_csv_files(mechanisms[2] + "_" + name + '.csv', ['t'] + ['d2']+ ["v2"] + ["a2"],[[t] + [p] + [v] + [a] for  t, p, v, a in zip(time2, position2, velocity2, acceleration2)])
  
  

  
      print(f'Calculation duration: {first_output.calculation_duration:0.1f} [µs]')
      print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} [s]')
  
