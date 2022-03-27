# CAN protocol compliant message, to be used to publish requests or received frames
#
# Number of bytes in 'data' array needs to NOT exceed 8, otherwise the message will be ignored.
# 
###################################################################################################

std_msgs/Header header

uint32      identifier      # Identifier for the CAN message
uint8       extendedMode    # Set to '0' for Standard Identifier
                            #        '1' for Extended Identifier

uint8[]     data