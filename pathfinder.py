import time
from typing import Any
from mcpi.minecraft import Minecraft
from mcpi import block
import math



mc=Minecraft.create()


## BLOCK CONSTANTS
# List of passable blocks (Mostly composed of vegetation that players can walk through i.e flowers,ferns,mushrooms)
AIR=block.AIR.id

PASSABLE_BLOCKS=[AIR,31,175,37,38,39,40,8,78]


class Pathfinder:
    def __init__(self,start_grid:tuple,end_grid:tuple) -> None:
        self.cost_grid=Pathfinder.generateGrid(start_grid,end_grid)
        self.path_list=[]
    def manhatten_distance(self,vector1:tuple,vector2:tuple)->int:
        
        
        x1,y1,z1=vector1
        x2,y2,z2=vector2

        deltaX=abs(x1-x2)
        deltaY=abs(y1-y2)
        deltaZ=abs(z1-z2) 

        # Apply penalty for floating blocks
        if self.cost_grid.get((x1,y1-1,z1))==0 or self.cost_grid.get((x2,y2-1,z2))==0:
            return  (deltaX + deltaY+ deltaZ)*8
        else:
            return deltaX + deltaY + deltaZ
        
    
        
        
    def heuristic(self,vector1:tuple,vector2:tuple)->int:
        x1,y1,z1=vector1
        x2,y2,z2=vector2
        
        deltaX=abs(x1-x2)
        deltaY=abs(y1-y2)
        deltaZ=abs(z1-z2)
        
         # Apply penalty for floating blocks
        if  self.cost_grid.get((x1,y1-1,z1))==0:
            return  (deltaX + deltaY + deltaZ)*4
        else:
            return deltaX + deltaY + deltaZ
        
        
    # Vector with components (x,y,z)
    # Returns a dictionary in format {(x,y,z):int}
  
    def generateGrid(start_point:tuple,end_point:tuple)->dict:
        cost_grid={}
        
        x1,y1,z1=start_point
        x2,y2,z2=end_point
        
        minX=min(x1,x2)
        minY=min(y1,y2)
        minZ=min(z1,z2)
        
        maxX=max(x1,x2)
        maxY=max(y1,y2)
        maxZ=max(z1,z2)
        
        x_toIterate=range(minX,maxX+1)
        y_toIterate=range(minY,maxY+1)
        z_toIterate=range(minZ,maxZ+1)
        
        #Ordered by y,x,z
        block_list=list(mc.getBlocks(minX,minY,minZ,maxX,maxY,maxZ))
        index=0
        # Iterate through the list of blocks 
        # int=0 , if traversable
        # int=1 , if non-traversable
        for y in y_toIterate:
            for x in x_toIterate:
                for z in z_toIterate:
                    if block_list[index] in PASSABLE_BLOCKS:
                        cost_grid[(int(x),int(y),int(z))]=0
                        index+=1
                    else:
                        cost_grid[(int(x),int(y),int(z))]=1
                        index+=1
        return cost_grid

    def find_neighbours(self,point:tuple)->list:
        neighbours=[]
        
        possible_directions=[-1,0,1]  
        
        x,y,z=point

        for deltaX in possible_directions:
            for deltaY in possible_directions:
                for deltaZ in possible_directions:
                    # Skip to next iteration if delta of x,y,z is 0
                    if deltaX == 0 and deltaY==0 and deltaZ==0: #CASE(0,0,0)
                        continue
                    
                    if deltaX!=0 and deltaY!=0 and deltaZ !=0: #CASE (X,Y,Z)
                        # Check the immediate block next to each axis if it's clear --> if it's clear move onto next case --> if not clear move to next iteration
                        if self.cost_grid.get((x+deltaX,y,z)) != AIR or  self.cost_grid.get((x,y+deltaY,z)) != AIR or self.cost_grid.get((x,y,z+deltaZ)) != AIR:
                            continue
                        if self.cost_grid.get((x+deltaX,y+deltaY,z)) != AIR or  self.cost_grid.get((x+deltaX,y,z+deltaZ)) != AIR or self.cost_grid.get((x,y+deltaY,z+deltaZ)) != AIR or self.cost_grid.get((x+deltaX,y+deltaY,z+deltaZ)) != AIR:
                            neighbours.append((x+deltaX,y+deltaY,z+deltaZ))
                    
                    elif deltaX!=0 and deltaY!=0:  # CASE (X,Y,0)
                        if self.cost_grid.get((x+deltaX,y,z)) != AIR or self.cost_grid.get((x,y+deltaY,z)) != AIR:
                            continue
                        if self.cost_grid[(x+deltaX,y+deltaY,z)] != AIR:
                            neighbours.append((x+deltaX,y+deltaY,z))
                    elif deltaX!=0 and deltaZ!=0:  # CASE (X,0,Z)
                        if self.cost_grid.get((x+deltaX,y,z)) != AIR or self.cost_grid.get((x,y,z+deltaZ)) != AIR:
                            continue
                        if self.cost_grid.get((x+deltaX,y,z+deltaZ)) != AIR:
                            neighbours.append((x+deltaX,y,z+deltaZ))
                            
                    elif deltaY!=0 and deltaZ!=0:  # CASE (0,Y,Z) 
                        if self.cost_grid.get((x,y+deltaY,z)) != AIR or self.cost_grid.get((x,y,z+deltaZ)) != AIR:
                            continue
                        if self.cost_grid.get((x,y+deltaY,z+deltaZ)) != AIR:
                            neighbours.append((x,y+deltaY,z+deltaZ))
                    else:
                        if deltaX !=0:    # CASE (X,0,0)
                            if self.cost_grid.get((x+deltaX,y,z)) != AIR:
                                continue
                            neighbours.append((x+deltaX,y,z))
                        if deltaY !=0:    # CASE (0,Y,0)
                            if self.cost_grid.get((x,y+deltaY,z)) != AIR:
                                continue
                            neighbours.append((x,y+deltaY,z))
                        if deltaZ !=0:    # CASE (0,0,Z)
                            if self.cost_grid.get((x,y,z+deltaZ)) != AIR:
                                continue
                            neighbours.append((x,y,z+deltaZ))
            
                    
        return neighbours



    def check_forced_neighbour(self,current_point:tuple,axis:tuple) -> bool:
        
        
        deltaX,deltaY,deltaZ=axis
        x1,y1,z1=current_point
        
        if deltaX!=0:
            # CASE(X,0,0)
            # Check above and below for forced neighbour along chosen axis 
            if self.cost_grid.get((x1+deltaX,y1,z1)) == 1  and self.cost_grid.get((x1+deltaX,y1+1,z1))==0 or self.cost_grid.get((x1+deltaX,y1,z1))==0 and self.cost_grid.get((x1,y1-1,z1))==0:
                return True
            
        if deltaY!=0:
            # CASE(0,Y,0)
            if self.cost_grid.get((x1,y1+deltaY,z1)) == 1  and self.cost_grid.get((x1,y1+deltaY,z1+1))==0 or self.cost_grid.get((x1,y1+deltaY,z1))==0 and self.cost_grid.get((x1,y1+deltaY,z1-1))==0:
                return True
            
        if deltaZ!=0:
            # CASE (0,0,Z)
            if self.cost_grid.get((x1,y1,z1+deltaZ)) == 1  and self.cost_grid.get((x1+1,y1,z1+deltaZ))==0 or self.cost_grid.get((x1,y1,z1+deltaZ))==0 and self.cost_grid.get((x1-1,y1+deltaY,z1+deltaZ))==0:
                return True
            
            
    def jump(self,current_point:tuple,goal_point:tuple,ancestor:dict):
        x1,y1,z1=current_point
        x2,y2,z2=goal_point
        
        deltaX=x2-x1
        deltaY=y2-y1
        deltaZ=z2-z1
        
        
    
        # Check if delta x , y , z is in ancestor if not add them to ancestor with (current_point) as the value
        if (deltaX,deltaY,deltaZ) not in ancestor:
            ancestor[(deltaX,deltaY,deltaZ)]=current_point

        # check if current_point == goal_point
        if (deltaX,deltaY,deltaZ) == (0,0,0):
            return (goal_point,ancestor)

        # Calculate maximum distance to jump to
        max_dist=max(abs(deltaX),abs(deltaY),abs(deltaZ))
        
        # Find the axis/direction to move in from delta x,y,z 
        # Same logic as the four cardinal directions --> Direction of travel is determined by the axis and the SIGN/Magnitiuide  (+,-)
        # Used to check for forced neighbour
        direction_vector=(math.copysign(1,deltaX) , math.copysign(1,deltaY) , math.copysign(1,deltaZ))
        
        
        # Check if current point is traversable 
        if self.cost_grid[current_point]>=1:
            return None

        # Check for forced neighbour
        if self.check_forced_neighbour(current_point,direction_vector):
            return (current_point,ancestor)


        # Iterate
        for index in range(1,int(max_dist)+1):
            deltaXYZ=(direction_vector[0]*index,direction_vector[1]*index,direction_vector[2]*index)
            next_point=(current_point[0]+deltaXYZ[0],current_point[1]+deltaXYZ[1],current_point[2]+deltaXYZ[2])

        if self.cost_grid.get(next_point)==1:
            return None
        if self.check_forced_neighbour(next_point,direction_vector):
            return (next_point,ancestor)

        return None

    def return_path(ancestor:dict,start_point:tuple,end_point:tuple)->list:
        
        path=[end_point]
        
        while path[-1] != start_point:
            path.append(ancestor[path[-1]])
        path.reverse()
        
        return path

    def jump_point_search(self,start_point:tuple,end_point:tuple):
        
        ancestor={}
        open_set={start_point}
        closed_set=set()
        
        path_distance={start_point:0}
        path_cost={start_point:self.heuristic(start_point,end_point)}
        
        while open_set:
            # Find the point in the open set with the lowest total cost (Sum of path_dsitance and path_cost)
            current_point = min(open_set,key=lambda point: path_distance[point] + path_cost[point])
        
            if current_point==end_point:
                return Pathfinder.return_path(ancestor,start_point,end_point)

            open_set.remove(current_point)
            closed_set.add(current_point)

        
            neighbours=self.find_neighbours(current_point)
            
            for neighbour in neighbours:
                # Skip the neighbouring node if it's already been examined (i.e. in the closed set)
                if neighbour in closed_set:
                    continue

                temp_cost = path_distance[current_point] + self.manhatten_distance(current_point,neighbour)
                
                # if the neighbour has not beene examined add it to the set to be examined  + calculate the path_cost
                if neighbour not in open_set:
                    open_set.add(neighbour)
                    path_cost[neighbour] = self.heuristic(neighbour,end_point)
                # if the cost of the current_point to the neighbour is higher than current cost for the neighbour in path_distance skip it
                elif temp_cost >= path_distance[neighbour]:
                    continue
                
                ## Update the ancestor and path_distance values for the current neighbour 
                ancestor[neighbour]=current_point
            
                path_distance[neighbour]=temp_cost
        
                result_tuple=self.jump(current_point,neighbour,ancestor)
            
                if result_tuple is not None:
                    jump_point,ancestor = result_tuple
                    if jump_point not in closed_set:
                        open_set.add(jump_point)
                        path_cost[jump_point] = self.heuristic(jump_point,end_point)
                        path_distance[jump_point] = path_distance[current_point] + self.manhatten_distance(current_point,jump_point)
    
        return None
    







if __name__=="__main__":
    
    # Variables defining , where the grid starts and ends and the start  and end point of the path
    START_OF_GRID=(-41,51,-29)
    END_OF_GRID=(110,117,87)
    
    START_POINT=(-30,69,27)
    END_POINT=(91,71,24)

    PATH_MATERIAL=89

    # Intialise class, with Grid coords to generate a cost grid
    Pathfind =  Pathfinder(START_OF_GRID,END_OF_GRID)
    # Ensure that the start and end points are reachable
    Pathfind.cost_grid[START_POINT]=0
    Pathfind.cost_grid[END_POINT]=0
    # Call jump point search function with start point and end point
    path_to_build=Pathfind.jump_point_search(START_POINT,END_POINT)

                
    # build the path found

    for vector in path_to_build:
            x, y, z = vector
            time.sleep(0.1)
            mc.setBlocks(x-1, y, z-1, x+1, y, z+1, PATH_MATERIAL)
    time.sleep(30)
    for vector in path_to_build:
            x, y, z = vector
            
            mc.setBlocks(x-1, y, z-1, x+1, y, z+1, AIR)

            
