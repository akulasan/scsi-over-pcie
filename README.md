scsi-over-pcie
==============

Linux driver for SCSI over PCIe devices

The block directory contains the block driver, and is where all
current development is happening.

The scsi directory contains an early scsi driver which formed
the basis of the block driver, but which was ultimately abandoned
due to poor performance relative to the block driver.

