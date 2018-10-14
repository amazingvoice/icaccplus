
import json


#def main():
    #ans = getconflictregion(1, "s", 2, "r")
    #print (ans[0][1])
    
    
class Data:
    
    def __init__(self):
        with open('intersection_info_v2.json', 'r') as file:
            self.tau = json.load(file)

    def getConflictRegion(self, l1, d1, l2, d2):

        # rotate to corresponding cases in Lane 1 & 2
        if l1 != 1 and l1 != 2:
            if l1 % 2 == 1:
                l2 = (l2 - (l1 - 1)) % 8
                l1 = 1
            else:
                l2 = (l2 - (l1 - 2)) % 8
                l1 = 2

        # Test if there is conflict between l1 & l2
        # and give the conflict region.
        

        if (str(l1) + d1 + str(l2) + d2) in self.tau:
            # lane 1 intersects with lane 2
            return self.tau[str(l1) + d1 + str(l2) + d2]
        else:
            return []



