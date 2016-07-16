# This assembly file is to fix a bug associated with the xmos
# compiler.  In certain situations, when a function is called
# within another function, we get the following error:
#
# `Error:   lower bound could not be calculated (function is recursive?).`
#
# This sets the size of the stack for the function so that the "lower
# bound can be calculated"
.globl SerialBufferPush.nstackwords
.linkset SerialBufferPush.nstackwords, 100
.globl SerialBufferSaveDouble.nstackwords
#.linkset SerialBufferSaveDouble.nstackwords, 100
.globl SerialBufferSaveInt.nstackwords
#.linkset SerialBufferSaveInt.nstackwords, 100
.globl SerialBufferSaveString.nstackwords
#.linkset SerialBufferSaveString.nstackwords, 100
.globl SerialBufferPrintfVargs.nstackwords
.linkset SerialBufferPrintfVargs.nstackwords, 100
.globl SerialBufferSaveChar.nstackwords
#.linkset SerialBufferSaveChar.nstackwords, 100
.globl SerialBufferForceSend.nstackwords
.linkset SerialBufferForceSend.nstackwords, 100
