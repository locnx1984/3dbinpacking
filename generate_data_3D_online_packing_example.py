#!/usr/bin/env python 
# Generate dataset for Online-3D Bin Packing
# prepare training data following 2021 paper: Online 3D Bin Packing with Constrained Deep Reinforcement Learning - Hang Zhao1, Qijin She1, Chenyang Zhu1, Yin Yang2, Kai Xu1
# Link: https://arxiv.org/pdf/2006.14978.pdf
# Authors: Loc Nguyen 
# Solomon Technology Corp.
# Copyright - 2021 
# 
# The software contains proprietary information of Solomon Technology Corp.  
# It is provided under a license agreement containing restrictions on use and disclosure 
# and is also protected by copyright law. Reverse engineering of the software is prohibited. 
# 
# No part of this publication may be reproduced, stored in a retrieval system, 
# or transmitted in any form or by any means, electronic, mechanical, photocopying, recording or otherwise 
# without the prior written permission of Solomon Technology Corp. 
#  
from py3dbp import Online3DPackingDataGenerator
#Start a test
generator=Online3DPackingDataGenerator()
generator.generate_data()
#press "N" or SPACE for Next Packing
#press "P" for Previous Packing
generator.display_current_result()