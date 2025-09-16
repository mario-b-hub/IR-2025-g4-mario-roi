import random

def leftsensor():
    leftreading=random.randint(2,200)
    return leftreading


def rightsensor():
    rightreading=random.randint(2,200)
    return rightreading

for i in range(20):
 
    leftreading=leftsensor()
    rightreading=rightsensor()

    if leftreading < 40 and rightreading < 40:
        print("stop")
    elif leftreading < 40:
        print("turn_right")
    elif rightreading < 40:
        print("turn_left")
    else:
        print("move_forward")

    #print(leftreading,rightreading)

