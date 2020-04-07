import numpy as np
import matplotlib.pyplot as pyplot

np.set_printoptions(threshold=10000)


#for i in range(0, len(rotBlock)):
#    world[int(rotBlock[i][0,0]+5)][int(rotBlock[i][0,1]+5)] = 1

def wavefront(world, goal):

    world[goal[0]][goal[1]] = 2

    wave = [goal]

    def inBounds(world, pos):
        isIn = pos[0] >= 0 and pos[0] < len(world) and pos[1] >= 0 and pos[1] < len(world[0])
        return isIn and world[pos[0]][pos[1]] == 0

    while len(wave) > 0:
        #print("wave len: {}".format(len(wave)))
        current = wave[0]
        wave = wave[1:]
        val = world[current[0]][current[1]]
        next = (current[0]+1, current[1])
        if inBounds(world, next):
            world[next[0]][next[1]] = val+1
            wave.append(next)
        next = (current[0] - 1, current[1])
        if inBounds(world, next):
            world[next[0]][next[1]] = val + 1
            wave.append(next)
        next = (current[0], current[1] + 1)
        if inBounds(world, next):
            world[next[0]][next[1]] = val + 1
            wave.append(next)
        next = (current[0], current[1] - 1)
        if inBounds(world, next):
            world[next[0]][next[1]] = val + 1
            wave.append(next)

        next = (current[0] + 1, current[1] - 1)
        if inBounds(world, next):
            world[next[0]][next[1]] = val + 1
            wave.append(next)
        next = (current[0] - 1, current[1] - 1)
        if inBounds(world, next):
            world[next[0]][next[1]] = val + 1
            wave.append(next)
        next = (current[0]+1, current[1] +1)
        if inBounds(world, next):
            world[next[0]][next[1]] = val + 1
            wave.append(next)
        next = (current[0]-1, current[1] + 1)
        if inBounds(world, next):
            world[next[0]][next[1]] = val + 1
            wave.append(next)

    return world

def planWave(world, goal, start):
    waved = wavefront(world, goal)
    #print("{}".format(waved[0:20, 20:40]))
    path = [start]
    def inBounds(world, pos):
        isIn = pos[0] >= 0 and pos[0] < len(world) and pos[1] >= 0 and pos[1] < len(world[0])
        return isIn and world[pos[0]][pos[1]] > 1

    def getMinSpace(world, pos):
        currentVal = world[pos[0]][pos[1]]
        out = pos
        next = (pos[0] + 1, pos[1])
        if inBounds(world, next):
            if currentVal > world[next[0]][next[1]]:
                currentVal = world[next[0]][next[1]]
                out = next

        next = (pos[0] - 1, pos[1])
        if inBounds(world, next):
            if currentVal > world[next[0]][next[1]]:
                currentVal = world[next[0]][next[1]]
                out = next
        next = (pos[0], pos[1] + 1)
        if inBounds(world, next):
            if currentVal > world[next[0]][next[1]]:
                currentVal = world[next[0]][next[1]]
                out = next
        next = (pos[0], pos[1] - 1)
        if inBounds(world, next):
            if currentVal > world[next[0]][next[1]]:
                currentVal = world[next[0]][next[1]]
                out = next

        next = (pos[0] + 1, pos[1] - 1)
        if inBounds(world, next):
            if currentVal > world[next[0]][next[1]]:
                currentVal = world[next[0]][next[1]]
                out = next

        next = (pos[0] - 1, pos[1] -1)
        if inBounds(world, next):
            if currentVal > world[next[0]][next[1]]:
                currentVal = world[next[0]][next[1]]
                out = next
        next = (pos[0] - 1, pos[1] + 1)
        if inBounds(world, next):
            if currentVal > world[next[0]][next[1]]:
                currentVal = world[next[0]][next[1]]
                out = next
        next = (pos[0] + 1, pos[1] + 1)
        if inBounds(world, next):
            if currentVal > world[next[0]][next[1]]:
                currentVal = world[next[0]][next[1]]
                out = next
        return out


    while path[-1][0] != goal[0] or path[-1][1] != goal[1]:
        next = getMinSpace(waved, path[-1])
        #print("adding: {} to get to {}".format(next, goal))
        path.append(next)

    return path

def get_paths(waypoints, world):
    waypoints = waypoints
    world2 = world.copy()
    path1 = planWave(world,waypoints[1, :], waypoints[0, :])
    path2 = planWave(world2,waypoints[2, :], waypoints[1, :])
    return path1, path2

def graph_path(path, world, l1, l2):
    xs = []
    ys = []
    for p in path:
        angle1 = p[0]*(np.pi/300)
        angle2 = p[1]*(np.pi/300)
        #x = l1*np.cos(angle1) + l2*np.cos(angle1+angle2)
        #y = l1*np.sin(angle1) + l2*np.sin(angle1+angle2)
        xs.append(angle1)
        ys.append(angle2)

    fig, ax = pyplot.subplots()

    pyplot.plot(xs, ys, '.b')
    pyplot.plot(world[:, 0]*np.pi/180, world[:, 1]*np.pi/180, '.r')
    #patch = patches.PathPatch(blocks[0], facecolor='orange')
    # ax.add_patch(patch)
    #ax.set_xlim(0, 72)
    #ax.set_ylim(0, 54)
    print(path)

    pyplot.show()
