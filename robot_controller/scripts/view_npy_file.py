import numpy as np
import pdb
AR = np.load("AR_tag_positions.npy")
ALVAR = np.load("temp_ar_tags.npy")

ALVAR_LIST = ALVAR.tolist()

for i in range(1,len(ALVAR_LIST)):
    if(i==1):
        ALVAR = np.asarray(ALVAR_LIST[i])
    else:
        ALVAR = np.vstack((ALVAR,np.asarray(ALVAR_LIST[i])))

AR = AR[1:]

# np.save("pts_in_world_frame.npy",AR)
# np.save("pts_in_alvar_frame.npy",ALVAR)




pdb.set_trace()
