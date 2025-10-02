fp = "/home/lucas/coursework/MTE544/Complete/laser_content_circle.csv"

data = []
with open(fp, 'r') as file:
    for line in file:
        data.append(line)

data[0] = ", ".join(["laser_" + str(n) for n in range(1, 720+1)] + ["angle_increment", "stamp"]) + "\n"
new_data = [row.replace("array('f', [", "").replace("])", "") for row in data]
 
with open(fp.replace("laser", "laser_filtered"), 'w+') as file:
    file.writelines(new_data)