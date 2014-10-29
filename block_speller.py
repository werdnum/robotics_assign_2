#!/usr/bin/env python

import os
import rospy
#import pickup_block
from assign2.srv import Pickup
from assign2.msg import Block

letter_to_position = {}

def vision_callback(data):
    if not data.letter in letter_to_position:
        letter_to_position[data.letter] = data.posX

class BlockSpeller(object):
    def __init__(self):
        self.tokens = tokens
        rospy.init_node('block_speller', anonymous=True)
        rospy.Subscriber('blocks', Block, vision_callback)
	rospy.wait_for_service('pickup_block')

    def spell_word(self, word):
        tokens = [c for c in word]

        # Handle Duplicates
        if len(tokens) != len(set(tokens)):
            print "We don't have duplicate blocks sorry"
            return

        # Ensure everything is in the dictionary
        if len(tokens) != len([t for t in tokens if t in letter_to_position]):
            print "Sorry I couldn't find the following blocks:"
            print '*', ','.join([t for t in tokens if not t in letter_to_position])
            return

        # Translate to token positions
        token_positions = [(t, letter_to_position[t]) for t in tokens]
    
        place = 0
        for t,x in token_positions:
            print "* Picking Up", t, "at", x
            try:
                pickup_block = rospy.ServiceProxy('pickup_block', Pickup)
                pickup_block(x, place)
            except rospy.ServiceException, e:
                print "Arm service call failed: %s" % e
                return

            place += 1
        print "Spelling completed for", word

def pickup_block_client():
    block_speller = BlockSpeller()
    while True:
        print letter_to_position
        word_to_spell = input('Please Enter A Word To Spell:')
        print letter_to_position
        block_speller.spell_word(word_to_spell)
        print "Please reset the blocks"
        letter_to_position = {}


if __name__ == '__main__':
    pickup_block_client()
