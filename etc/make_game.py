SUFFICIENT_ALONE = 3
SUFFICIENT_CONN  = 2
SUFFICIENT_MSG   = 1

icons = [] # ID : list of IconConnections.
trans = dict(UP=0, RIGHT=1, DOWN=2, LEFT=3)
trans_rev = ['U', 'R', 'D', 'L']
icon_ids = dict() # Lookup for NAME : ID
id_icons = dict()
LISTEN_FLAG = 254

# TODO: OTHER ARM ID ISN'T THIS. IT'S SUPPOSED TO BE USED WITH
#  SUFFICIENT_CONN

# TODO: LISTEN_FLAG is never getting set

class IconConnection(object):
    def __init__(self, mate_icon, result_icon, sufficiency, 
                 other_arm_id, arm_anim_id=0):
        self.mate_icon = mate_icon
        self.result_icon = result_icon
        self.sufficiency = sufficiency
        self.arm_anim_id = arm_anim_id
        self.other_arm_id = other_arm_id

def get_icons():
    with open('icons.csv') as icon_file:
        icon_file.readline() # Burn the headers
        while 1:
            record_str = icon_file.readline()
            if not record_str:
                break
            record = record_str.strip().split(',')
            icon_ids[record[1]] = int(record[3])-1 #make it 0-origined
            id_icons[int(record[3])-1] = record[1]
            
    for i in sorted(icon_ids.keys(), cmp=lambda x,y: cmp(icon_ids[x], icon_ids[y])):
        icons.append([None, None, None, None])
            
    with open('icons_adjacency.csv') as adjacency_file:
        adjacency_file.readline() # Burn the headers.
        while 1:
            record_str = adjacency_file.readline()
            if not record_str:
                break
            record = record_str.upper().strip().split(',')
            if record[1] == 'FREEZER' or record[1] == 'BRIGHT LIGHT':
                continue # Procedurally in code. Special cases.
            if not record[2]:
                continue # No transition here.
            this_icon = icon_ids[record[0]]
            this_dir = trans[record[1]]
            if record[7] == 'PARTIAL': # sufficient_msg
                # This is a connection where our partner must be
                #  the one to connect to the third badge.
                adj = IconConnection(
                    icon_ids[record[2]], # Mate 1
                    LISTEN_FLAG, # result placeholder
                    SUFFICIENT_MSG,
                    (trans[record[1]] + 2) % 4 # mate's direction
                )
                mate_con = icons[icon_ids[record[2]]][adj.other_arm_id]
                if mate_con:
                    # Validate the other side.
                    assert mate_con.mate_icon == this_icon
                    assert mate_con.other_arm_id == this_dir
                    assert mate_con.sufficiency == SUFFICIENT_CONN
                    
                icons[this_icon][this_dir] = adj
            elif record[3]: # sufficient_conn
                # This is a connection where we'll be the one
                #  to attach to the third badge.
                adj1 = IconConnection(
                    icon_ids[record[2]], # Mate 1
                    icon_ids[record[7]], # result
                    SUFFICIENT_CONN,
                    (this_dir + 2) % 4 # mate's direction
                )
                mate_con1 = icons[adj1.mate_icon][adj1.other_arm_id]
                
                if mate_con1:
                    assert mate_con1.mate_icon == this_icon
                    assert mate_con1.other_arm_id == this_dir
                    assert mate_con1.sufficiency == SUFFICIENT_MSG
                
                this_dir2 = trans[record[3]]
                adj2 = IconConnection(
                    icon_ids[record[4]], # Mate 1
                    icon_ids[record[7]], # result
                    SUFFICIENT_CONN,
                    (this_dir2 + 2) % 4 # mate's direction
                )
                mate_con2 = icons[adj2.mate_icon][adj2.other_arm_id]
                
                if mate_con2:
                    assert mate_con2.mate_icon == this_icon
                    assert mate_con2.other_arm_id == this_dir2
                    assert mate_con2.sufficiency == SUFFICIENT_MSG
                
                
                icons[this_icon][this_dir] = adj1
                icons[this_icon][this_dir2] = adj2
                
            else: # sufficient_alone
                adj = IconConnection(
                    icon_ids[record[2]], # Mate 1
                    icon_ids[record[7]], # result
                    SUFFICIENT_ALONE,
                    (trans[record[1]] + 2) % 4 # mate's direction
                )
                mate_con = icons[icon_ids[record[2]]][adj.other_arm_id]
                if mate_con:
                    # Validate the other side.
                    assert mate_con.mate_icon == this_icon
                    assert mate_con.result_icon == adj.result_icon
                    assert mate_con.other_arm_id == this_dir
                    assert mate_con.sufficiency == SUFFICIENT_ALONE

                icons[this_icon][this_dir] = adj
                
    # TODO: Confirm what queercon down game looks like.
    # TODO: Add colors
                            
    for icon_id in range(len(icons)):
        icon = icons[icon_id]
        print id_icons[icon_id]
        arm_id = 0
        for connection in icon:
            arm_id += 1
            if not connection:
                continue
            print '\t' + trans_rev[arm_id-1],
            print id_icons[connection.mate_icon], 
            if connection.sufficiency == SUFFICIENT_ALONE:
                print '=', 
                print id_icons[connection.result_icon]
            elif connection.sufficiency == SUFFICIENT_CONN:
                print ': CAN CONNECT TO MAKE',
                print id_icons[connection.result_icon]
            elif connection.sufficiency == SUFFICIENT_MSG:
                print ': OTHER BADGE COMPLETES THIS.'
                
    return icons