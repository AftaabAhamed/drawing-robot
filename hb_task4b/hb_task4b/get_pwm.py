import numpy as np
class pwm_provider():
    def __init__(self,path_list) -> None:

        file21 = open(path_list[0], 'r')
        file22 = open(path_list[1], 'r')
        file23 = open(path_list[2], 'r')
        m21 = []
        m22 = []
        m23 = []

        maps = [m21,m22,m23] 
        files = [file21,file22,file23]
        count = 0
        for m,file in zip(maps,files): 
            while True:
                count += 1
                line = file.readline()                
                if not line:
                    break
                line  = line.strip()
                pwm , vel = line.split()
                m.append([float(vel)])
                
            m = np.asarray(m)
            file.close()

        m21 = np.asarray(m21)
        m22 = np.asarray(m22)
        m23 = np.asarray(m23)
        maps = [m21,m22,m23]
        minlist = [] 
        maxlist = []
        for m in maps:
            minlist.append(m[:90].min())
            maxlist.append(m[90:].max())
        
        min_ = max(minlist)
        max_ = min(maxlist)

        for m in maps:
            m[:90] = (m[:90]/abs(min_))
            m[90:] = (m[90:]/abs(max_))

        self.maps = maps


    def get_pwm(self,ik):
        m1  = self.maps[0]
        m2  = self.maps[1]
        m3  = self.maps[2]
        
        diff = abs(m1 - (ik[0]))
        pwm1 = diff.argmin() + 1
        diff = abs(m2 - (ik[1]))
        pwm2 = diff.argmin() + 1 
        diff = abs(m3 - (ik[2]))
        pwm3 = diff.argmin() + 1 
        if ik[0] == 0:
            pwm1 = 90
        if ik[1] == 0:
            pwm2 = 90
        if ik[2] == 0:
            pwm3 = 90

        return float(pwm1) ,float(pwm2), float(pwm3)

