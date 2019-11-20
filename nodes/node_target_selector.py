
#I set the gain of each class of targets as 0.8, 0.5, 0.2, the target, whose distance
# is less than 1 meter between a robot, will get a value of priority. 
# Priorities are 0.25-->[1.5 meter,1 m) 0.5-->[1, 0.75) 0.75-->[0.75,0.5) 1-->[0.5,0.25)
# Our sonor safety control will set a saftey distance as 0.1, if the distance is smaller
# than 0.1 meter, the interception is suscessful.

class CognitiveSelector:

    # pedestrian = 0.8 ---> p
    # miro = 0.5 ---> m
    # ball = 0.2 ---> b
    def __int__(self):
        p_gain =0.8
        m_gain = 0.5
        b_gain = 0.2
        p_distance = 0
        m_distance = 0
        b_distance = 0

    def select_target(self,p_distance, m_distance, b_distance):
        p_priority =  self.get_priority(p_distance)
        m_priority =  self.get_priority(m_distance)
        b_priority =  self.get_priority(b_distance)
        p_list = []
        p_list.append(p_priority)
        p_list.append(m_priority)
        p_list.append(b_priority)
        max_num = 0
        for i in range(len(p_list)):
            if p_list[i] > max_num:
                max_num = i
        return max_num

    def get_priority(self,distance):
        if distance > 1 and distance <= 1.5:
            priority = 0.25
        elif distance > 0.75 and distance <= 1:
            priority = 0.5
        elif distance > 0.5 and distance <= 0.75:
            priority = 0.75
        elif distance > 0.25 and distance <= 0.5:
            priority = 1
        return priority
