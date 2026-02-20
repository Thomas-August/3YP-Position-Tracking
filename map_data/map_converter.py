import openpyxl
from collections import defaultdict
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(script_dir, 'Map.xlsx')

wb = openpyxl.load_workbook(file_path)
grid = defaultdict(int)

def is_wall(side):
    if not side.style:
        return False
    
    try:
        if side.color and side.color.rgb == 'FFCCCCCC':
            return False
    except:
        return True
        
    return True

for row in wb.active.iter_rows():
    for c in row:
        r, col = c.row, c.column
        print("row: " + str(r), " col: " + str(col))
        if is_wall(c.border.top):   
            print("top") 
            grid[(r, col)]   |= 1
        if is_wall(c.border.left):   
            print("left")
            grid[(r, col)]   |= 2
        if is_wall(c.border.right): 
            print("right") 
            grid[(r, col+1)] |= 2
        if is_wall(c.border.bottom): 
            print("bottom")
            grid[(r+1, col)] |= 1

if grid:
    max_r, max_c = max(k[0] for k in grid), max(k[1] for k in grid)
    with open(os.path.join(script_dir, 'map_data.csv'), 'w') as f:
        for r in range(1, max_r + 1):
            f.write(','.join([str(grid[(r, c)]) for c in range(1, max_c + 1)]) + '\n')