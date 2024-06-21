###--- Place ---###
pl_observation = {'type': 'joint', 'pose': [-0.4136 , 0.0279 , 1.6563 , 0.6387 , 1.7428, 1.1938]}

pl_home = {'type': 'linear', 'pose': [282.0, 187.9, 404.5, 3.1416, 0.0, -1.5498]}

pl_pal2home = [{'type': 'joint', 'pose': [0.5881, 0.5253, 1.9373, 0.0, 1.4102, -0.8587], 'gripper' : 'NA'}, #CAMARA
            {'type': 'joint', 'pose': [2.7785, 1.0995, 2.4120, 0.0, 1.3124, -0.3438], 'gripper' : 'NA'}, #ADENTRO
            {'type': 'linear', 'pose': [-395.1, 149.5, 122.7, 3.1416, 0.0, 3.1206], 'gripper' : 'close'}, #BAJAR
            {'type': 'linear', 'pose': [-395.1, 149.5, 314.1, 3.1416, 0.0, 3.1206], 'gripper' : 'NA'},
            {'type': 'linear', 'pose': [282.0, 187.9, 404.5, 3.1416, 0.0, -1.5498], 'gripper' : 'NA'}] #Home

pl_2pal2home = [{'type': 'joint', 'pose': [0.5881, 0.5253, 1.9373, 0.0, 1.4102, -0.8587], 'gripper' : 'NA'}, #CAMARA
            {'type': 'joint', 'pose': [2.7785, 1.0995, 2.4120, 0.0, 1.3124, -0.3438], 'gripper' : 'NA'}, #ADENTRO
            {'type': 'linear', 'pose': [-395.1, 149.5, 122.7, 3.1416, 0.0, 3.1206], 'gripper' : 'close'}, #BAJAR
            {'type': 'linear', 'pose': [-395.1, 149.5, 314.1, 3.1416, 0.0, 3.1206], 'gripper' : 'NA'},
            {'type': 'linear', 'pose': [282.0, 187.9, 404.5, 3.1416, 0.0, -1.5498], 'gripper' : 'NA'}] #Home

pl_pallets = [pl_pal2home , pl_2pal2home]

pl_full = [{'type': 'linear', 'pose': [282.0, 187.9, 404.5, 3.1416, 0.0, -1.5498], 'gripper' : 'open'}, #HOME
            {'type': 'joint', 'pose': [0.5881, 0.5253, 1.9373, 0.0, 1.4102, -0.8587], 'gripper' : 'NA'}, #CAMARA
            {'type': 'joint', 'pose': [2.7785, 1.0995, 2.4120, 0.0, 1.3124, -0.3438], 'gripper' : 'NA'}, #ADENTRO
            {'type': 'linear', 'pose': [-395.1, 149.5, 122.7, 3.1416, 0.0, 3.1206], 'gripper' : 'close'}, #BAJAR
            #CERRAR GRIPPER
            {'type': 'linear', 'pose': [-395.1, 149.5, 314.1, 3.1416, 0.0, 3.1206], 'gripper' : 'NA'}, #SUBIR

            {'type': 'joint', 'pose': [0.2443, 0.1483, 1.1798, 0.0, 1.0314, 1.9250], 'gripper' : 'NA'}, #ARRIBA DEL P1

            {'type': 'linear', 'pose': [261.4, 65.3, 212.4, 3.1416, 0.0, -1.6790], 'gripper' : 'open'}, #BAJAR
            #ABRIR GRIPPER P1
            {'type': 'linear', 'pose': [261.4, 65.3, 348.8, 3.1416, 0.0, -1.6790], 'gripper' : 'NA'}, #SUBIR
            {'type': 'linear', 'pose': [282.0, 187.9, 404.5, 3.1416, 0.0, -1.5498], 'gripper' : 'NA'},]

###--- Pick ---###
pi_observation = {'type': 'joint', 'pose': [-0.0785 , -0.0226 , 1.7034 , -0.6335 , 1.6650 , -1.6423]}

pi_home = {'type': 'linear', 'pose': [309.4, -134.8, 321.1, 3.1416, 0.0, -1.6458]}

pi_home2pallet = [{'type': 'linear', 'pose': [309.4, -134.8, 321.1, 3.1416, 0.0, -1.6458] , 'gripper' : 'NA'}, #Home
            {'type': 'joint', 'pose': [-2.7349, 1.1222, 2.5411, -0.0226, 1.4137, 0.4572] , 'gripper' : 'NA'}, #ADENTRO
            {'type': 'linear', 'pose': [-389.2, -158.9, 44.5, 3.1241, -0.0157, 3.0927] , 'gripper' : 'open'}, #BAJAR a la base
            {'type': 'linear', 'pose': [-386.1, -161.7, 226.1, 3.1276, -0.0174, 3.0839] , 'gripper' : 'NA'}, #Subir
            {'type': 'linear', 'pose': [309.4, -134.8, 321.1, 3.1416, 0.0, -1.6458] , 'gripper' : 'NA'}] #Home

pi_home2pallet2 = [{'type': 'linear', 'pose': [309.4, -134.8, 321.1, 3.1416, 0.0, -1.6458] , 'gripper' : 'NA'}, #Home
            {'type': 'joint', 'pose': [-2.7349, 1.1222, 2.5411, -0.0226, 1.4137, 0.4572] , 'gripper' : 'NA'}, #ADENTRO
            {'type': 'linear', 'pose': [-389.2, -158.9, 44.5, 3.1241, -0.0157, 3.0927] , 'gripper' : 'open'}, #BAJAR a la base
            {'type': 'linear', 'pose': [-386.1, -161.7, 226.1, 3.1276, -0.0174, 3.0839] , 'gripper' : 'NA'}, #Subir
            {'type': 'linear', 'pose': [309.4, -134.8, 321.1, 3.1416, 0.0, -1.6458] , 'gripper' : 'NA'}] #Home

pi_pallets = [pi_home2pallet , pi_home2pallet2]

pi_full = [{'type': 'linear', 'pose': [309.4, -134.8, 321.1, 3.1416, 0.0, -1.6458], 'gripper' : 'open'}, #HOME
            {'type': 'joint', 'pose': [-0.5672, 0.4398, 1.5865, 0.0, 1.1466, 1.0786] , 'gripper' : 'NA'}, #ALINEAR TOOL
            {'type': 'linear', 'pose': [276.9, -176.6, 125.6, 3.1416, 0.0, -1.6498] , 'gripper' : 'close'}, #BAJAR
            #CERRAR GRIPPER
            {'type': 'linear', 'pose': [276.9, -176.6, 282.1, 3.1416, 0.0, -1.6498] ,'gripper' : 'NA'}, #SUBIR
            {'type': 'joint', 'pose': [-2.7349, 1.1222, 2.5411, -0.0226, 1.4137, 0.4572] , 'gripper' : 'NA'}, #ADENTRO
            {'type': 'linear', 'pose': [-389.2, -158.9, 44.5, 3.1241, -0.0157, 3.0927] , 'gripper' : 'open'}, #BAJAR A LA BASE
            #ABRIR GRIPPER
            {'type': 'linear', 'pose': [-386.1, -161.7, 226.1, 3.1276, -0.0174, 3.0839] , 'gripper' : 'NA'}, #SUBIR DE LA BASE
            {'type': 'linear', 'pose': [309.4, -134.8, 321.1, 3.1416, 0.0, -1.6458] , 'gripper' : 'NA'}] #Home
