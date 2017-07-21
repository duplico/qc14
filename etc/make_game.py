SUFFICIENT_ALONE = 3
SUFFICIENT_CONN  = 2
SUFFICIENT_MSG   = 1

icons = [] # ID : list of IconConnections.
trans = dict(UP=0, RIGHT=3, DOWN=2, LEFT=1)
trans_rev = ['U', 'L', 'D', 'R']
icon_ids = dict() # Lookup for NAME : ID
id_icons = dict()
LISTEN_FLAG = 254

ICON = 0
ICON_DIR1 = 1
ICON_MATE1 = 5
ICON_DIR1R = 2
ICON_DIR1G = 3
ICON_DIR1B = 4
ICON_DIR2 = 6
ICON_MATE2 = 10
ICON_DIR2R = 7
ICON_DIR2G = 8
ICON_DIR2B = 9
ICON_RESULT = 11

# TODO: OTHER ARM ID ISN'T THIS. IT'S SUPPOSED TO BE USED WITH
#  SUFFICIENT_CONN

# TODO: LISTEN_FLAG is never getting set

class IconConnection(object):
    def __init__(self, mate_icon, result_icon, sufficiency=SUFFICIENT_ALONE, 
                 other_arm_id=0, arm_anim_id=0, rgb=(0,0,0)):
        self.mate_icon = mate_icon
        self.result_icon = result_icon
        self.sufficiency = sufficiency
        self.arm_anim_id = arm_anim_id
        self.other_arm_id = other_arm_id
        self.rgb = map(int,rgb)

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
            if record[ICON_DIR1] == 'FREEZER' or record[ICON_DIR1] == 'BRIGHT LIGHT':
                continue # Procedurally in code. Special cases.
            if not record[ICON_MATE1]:
                continue # No transition here.
            this_icon = icon_ids[record[ICON]]
            this_dir = trans[record[ICON_DIR1]]
            rev_dir = (this_dir + 2) % 4
            print record
            if record[ICON_RESULT] == 'PARTIAL': # sufficient_msg
                # This is a connection where our partner must be
                #  the one to connect to the third badge.
                adj = IconConnection(
                    icon_ids[record[ICON_MATE1]], # Mate 1
                    LISTEN_FLAG, # result placeholder
                    sufficiency=SUFFICIENT_MSG,
                    rgb=(record[ICON_DIR1R],record[ICON_DIR1G],record[ICON_DIR1B])
                )
                mate_con = icons[icon_ids[record[ICON_MATE1]]][rev_dir]
                if mate_con:
                    # Validate the other side.
                    assert mate_con.mate_icon == this_icon
                    # assert mate_con.sufficiency == SUFFICIENT_CONN
                    assert mate_con.rgb == adj.rgb
                    
                icons[this_icon][this_dir] = adj
            elif record[ICON_DIR2]: # sufficient_conn
                # This is a connection where we'll be the one
                #  to attach to the third badge.
                this_dir2 = trans[record[ICON_DIR2]]
                rev_dir2 = (this_dir2 + 2) % 4
                
                adj1 = IconConnection(
                    icon_ids[record[ICON_MATE1]], # Mate 1
                    icon_ids[record[ICON_RESULT]], # result
                    sufficiency=SUFFICIENT_CONN,
                    other_arm_id=this_dir2, # The other side that needs to be connected
                    rgb=(record[ICON_DIR1R],record[ICON_DIR1G],record[ICON_DIR1B])
                )
                mate_con1 = icons[adj1.mate_icon][rev_dir]
                
                if mate_con1:
                    assert mate_con1.mate_icon == this_icon
                    assert mate_con1.sufficiency == SUFFICIENT_MSG
                    assert mate_con1.rgb == adj1.rgb
                
                adj2 = IconConnection(
                    icon_ids[record[ICON_MATE2]], # Mate 2
                    icon_ids[record[ICON_RESULT]], # result
                    sufficiency=SUFFICIENT_CONN,
                    other_arm_id=this_dir, # Other side for connect/completing
                    rgb=(record[ICON_DIR2R],record[ICON_DIR2G],record[ICON_DIR2B])
                )
                mate_con2 = icons[adj2.mate_icon][rev_dir2]
                
                if mate_con2:
                    assert mate_con2.mate_icon == this_icon
                    assert mate_con2.sufficiency == SUFFICIENT_MSG
                    assert mate_con2.rgb == adj2.rgb
                
                icons[this_icon][this_dir] = adj1
                icons[this_icon][this_dir2] = adj2
                
            else: # sufficient_alone
                adj = IconConnection(
                    icon_ids[record[ICON_MATE1]], # Mate 1
                    icon_ids[record[ICON_RESULT]], # result
                    SUFFICIENT_ALONE,
                    rgb=(record[ICON_DIR1R],record[ICON_DIR1G],record[ICON_DIR1B])
                )
                mate_con = icons[icon_ids[record[ICON_MATE1]]][rev_dir]
                if mate_con:
                    # Validate the other side.
                    assert mate_con.mate_icon == this_icon
                    assert mate_con.result_icon == adj.result_icon
                    assert mate_con.sufficiency == SUFFICIENT_ALONE
                    assert mate_con.rgb == adj.rgb

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