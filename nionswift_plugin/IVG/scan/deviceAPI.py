#Memory space for the AXI devices
# The onboard DDR2 RAM is accessible starting at this address.
RamBase = 0x0A000000
RamBase2 = 0x0C000000
RamBase3 = 0x0E000000

AD5543   = 0x80000000
AD9225   = 0x80010000
MAGBOARD = 0x80030000
DmaAxi0  = 0x80070000
DmaAxi1  = 0x80080000
DmaAxi2  = 0x80020000

MEMORY_BUFFER_LENGTH = 0x100000
NUMBER_OF_BUFFERS = 1

# 1st channel
S2MM_BUFFER_INITIAL = RamBase
S2MM_BUFFER_FINAL = S2MM_BUFFER_INITIAL + NUMBER_OF_BUFFERS * 0x40
S2MM_MEMORY_INITIAL = RamBase + 0x1000
S2MM_MEMORY_FINAL = S2MM_MEMORY_INITIAL + NUMBER_OF_BUFFERS * MEMORY_BUFFER_LENGTH

# 2nd channel
S2MM_BUFFER_INITIAL2 = RamBase2
S2MM_BUFFER_FINAL2 = S2MM_BUFFER_INITIAL2 + NUMBER_OF_BUFFERS * 0x40
S2MM_MEMORY_INITIAL2 = RamBase2 + 0x1000
S2MM_MEMORY_FINAL2 = S2MM_MEMORY_INITIAL2 + NUMBER_OF_BUFFERS * MEMORY_BUFFER_LENGTH

# 3rd channel
S2MM_BUFFER_INITIAL3 = RamBase3
S2MM_BUFFER_FINAL3 = S2MM_BUFFER_INITIAL3 + NUMBER_OF_BUFFERS * 0x40
S2MM_MEMORY_INITIAL3 = RamBase3 + 0x1000
S2MM_MEMORY_FINAL3 = S2MM_MEMORY_INITIAL3 + NUMBER_OF_BUFFERS * MEMORY_BUFFER_LENGTH


##### Registers for the Scatter / Gather mode #####
# MM2S CONTROL
MM2S_CONTROL_REGISTER = 0x00  # MM2S_DMACR
MM2S_STATUS_REGISTER = 0x04  # MM2S_DMASR
MM2S_CURDESC = 0x08  # must align 0x40 addresses
MM2S_CURDESC_MSB = 0x0C  # unused with 32bit addresses
MM2S_TAILDESC = 0x10  # must align 0x40 addresses
MM2S_TAILDESC_MSB = 0x14  # unused with 32bit addresses
SG_CTL = 0x2C  # CACHE CONTROL

# S2MM CONTROL
S2MM_CONTROL_REGISTER = 0x30  # S2MM_DMACR
S2MM_STATUS_REGISTER = 0x34  # S2MM_DMASR
S2MM_CURDESC = 0x38  # must align 0x40 addresses
S2MM_CURDESC_MSB = 0x3C  # unused with 32bit addresses
S2MM_TAILDESC = 0x40  # must align 0x40 addresses
S2MM_TAILDESC_MSB = 0x44  # unused with 32bit addresses

##### Registers for the direct mode #####
# MM2S CONTROL
# MM2S_CONTROL_REGISTER =      0x00 //Same as SG, defined above
# MM2S_STATUS_REGISTER =       0x04 //Same as SG, defined above
MM2S_SOURCE_ADDRESS = 0x18
MM2S_SOURCE_ADDRESS_MSB = 0x1C
MM2S_LENGTH = 0x28

# S2MM CONTROL
# S2MM_CONTROL_REGISTER =      0x30 //Same as SG, defined above
# S2MM_STATUS_REGISTER =       0x34 //Same as SG, defined above
S2MM_DEST_ADDRESS = 0x48
S2MM_DEST_ADDRESS_MSB = 0x4C
S2MM_LENGTH = 0x58

from . import udk3api
import sys, array, time, logging, numpy, threading

class WriteReadIOCesys():
    def __init__(self, d: udk3api.Device, address):
        self.device = d
        self.address = address

    def write_u32(self, offset_address, value):
        self.device.writeRegister(self.address + offset_address, value)

    def read_u32(self, offset_address):
        return self.device.readRegister(self.address + offset_address)

class AD5543_AXI(WriteReadIOCesys):
    def set_value(self, value):
        self.write_u32(0x00, value)

    def start_transfer(self, value: bool):
        self.write_u32(0x04, int(value))

    def set_mask(self, mask: str):
        assert len(mask) == 4 #This is 4 channel AD5543
        self.write_u32(0x08, int(mask, 2))

class MagBoard_AXI(WriteReadIOCesys):
    def set_value(self, value):
        self.write_u32(0x00, value)

    def start_transfer(self, value: bool):
        self.write_u32(0x04, int(value))

    def set_mask(self, mask: str):
        assert len(mask) == 6  # This is 6 channel AD5543
        self.write_u32(0x08, int(mask, 2))

    def set_switches(self, mask: str):
        assert len(mask) == 6  # This is 6 channel switches to control
        self.write_u32(0x0C, int(mask, 2))

class AD9225_AXI(WriteReadIOCesys):
    def set_filter_value(self, value):
        self.write_u32(0x00, value)


class DMA_AXI(WriteReadIOCesys):
    def start_dma(self, direction):
        initial_value = self.get_control_register(direction)

        if direction == "mm2s":
            self.write_u32(MM2S_CONTROL_REGISTER, 0x01 | initial_value)
        elif direction == "s2mm":
            self.write_u32(S2MM_CONTROL_REGISTER, 0x01 | initial_value)
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")

    def reset_dma(self, direction):
        initial_value = self.get_control_register(direction)

        if direction == "mm2s":
            self.write_u32(MM2S_CONTROL_REGISTER, 0x04 | initial_value)
        elif direction == "s2mm":
            self.write_u32(S2MM_CONTROL_REGISTER, 0x04 | initial_value)
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")

    def halt_dma(self, direction):
        initial_value = self.get_control_register(direction)

        if direction == "mm2s":
            self.write_u32(MM2S_CONTROL_REGISTER, 0x00 | initial_value)
        elif direction == "s2mm":
            self.write_u32(S2MM_CONTROL_REGISTER, 0x00 | initial_value)
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")

    def enable_interrupt(self, direction):
        initial_value = self.get_control_register(direction)

        if direction == "mm2s":
            self.write_u32(MM2S_CONTROL_REGISTER, 0x7000 | initial_value)
        elif direction == "s2mm":
            self.write_u32(S2MM_CONTROL_REGISTER, 0x7000 | initial_value)
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")

    def enable_cyclic_operation(self, direction):
        initial_value = self.get_control_register(direction)

        if direction == "mm2s":
            self.write_u32(MM2S_CONTROL_REGISTER, 0x10 | initial_value)
        elif direction == "s2mm":
            self.write_u32(S2MM_CONTROL_REGISTER, 0x10 | initial_value)
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")

    def set_current_descriptor_address(self, direction, value):
        if direction == "mm2s":
            self.write_u32(MM2S_CURDESC, value)
        elif direction == "s2mm":
            self.write_u32(S2MM_CURDESC, value)
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")

    def get_current_descriptor_address(self, direction):
        if direction == "mm2s":
            return self.read_u32(MM2S_CURDESC)
        elif direction == "s2mm":
            return self.read_u32(S2MM_CURDESC)
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")

    def set_tail_descriptor_address(self, direction, value):
        if direction == "mm2s":
            self.write_u32(MM2S_TAILDESC, value)
        elif direction == "s2mm":
            self.write_u32(S2MM_TAILDESC, value)
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")

    def get_status_register(self, direction):
        if direction == "mm2s":
            return self.read_u32(MM2S_STATUS_REGISTER)
        elif direction == "s2mm":
            return self.read_u32(S2MM_STATUS_REGISTER)
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")

    def get_control_register(self, direction):
        if direction == "mm2s":
            return self.read_u32(MM2S_CONTROL_REGISTER)
        elif direction == "s2mm":
            return self.read_u32(S2MM_CONTROL_REGISTER)
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")

    def get_status_verbose(self, direction):
        if direction == "mm2s":
            status = self.read_u32(MM2S_STATUS_REGISTER)
            if (status & 0x00000001):
                logging.warning(f"***AXI DMA***: MM2S system HALTED")
            if (status & 0x00000002):
                logging.warning(f"***AXI DMA***: MM2S system IDLE")
            if (status & 0x00000008):
                logging.warning(f"***AXI DMA***: MM2S system SGIncld")
            if (status & 0x00000010):
                logging.warning(f"***AXI DMA***: MM2S system DMAIntErr")
            if (status & 0x00000020):
                logging.warning(f"***AXI DMA***: MM2S system DMASlvEr")
            if (status & 0x00000040):
                logging.warning(f"***AXI DMA***: MM2S system DMADecErr")
            if (status & 0x00000100):
                logging.warning(f"***AXI DMA***: MM2S system SGIntErr")
            if (status & 0x00000200):
                logging.warning(f"***AXI DMA***: MM2S system SGSlvErr")
            if (status & 0x00000400):
                logging.warning(f"***AXI DMA***: MM2S system SGDecErr")
            if (status & 0x00001000):
                logging.warning(f"***AXI DMA***: MM2S system IOC_Irq")
            if (status & 0x00002000):
                logging.warning(f"***AXI DMA***: MM2S system Dly_Irq")
            if (status & 0x00004000):
                logging.warning(f"***AXI DMA***: MM2S system Err_Irq")
        elif direction == "s2mm":
            status = self.read_u32(S2MM_STATUS_REGISTER)
            if (status & 0x00000001):
                logging.warning(f"***AXI DMA***: S2MM system HALTED")
            if (status & 0x00000002):
                logging.warning(f"***AXI DMA***: S2MM system IDLE")
            if (status & 0x00000008):
                logging.warning(f"***AXI DMA***: S2MM system SGIncld")
            if (status & 0x00000010):
                logging.warning(f"***AXI DMA***: S2MM system DMAIntErr")
            if (status & 0x00000020):
                logging.warning(f"***AXI DMA***: S2MM system DMASlvEr")
            if (status & 0x00000040):
                logging.warning(f"***AXI DMA***: S2MM system DMADecErr")
            if (status & 0x00000100):
                logging.warning(f"***AXI DMA***: S2MM system SGIntErr")
            if (status & 0x00000200):
                logging.warning(f"***AXI DMA***: S2MM system SGSlvErr")
            if (status & 0x00000400):
                logging.warning(f"***AXI DMA***: S2MM system SGDecErr")
            if (status & 0x00001000):
                logging.warning(f"***AXI DMA***: S2MM system IOC_Irq")
            if (status & 0x00002000):
                logging.warning(f"***AXI DMA***: S2MM system Dly_Irq")
            if (status & 0x00004000):
                logging.warning(f"***AXI DMA***: S2MM system Err_Irq")
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")

        return status

    def get_control_verbose(self, direction):
        if direction == "mm2s":
            status = self.read_u32(MM2S_CONTROL_REGISTER)
            if (status & 0x00000001):
                logging.warning(f"***AXI DMA***: MM2S system Running")
            if (status & 0x00000004):
                logging.warning(f"***AXI DMA***: MM2S system Reset in progress")
            if (status & 0x0000008):
                logging.warning(f"***AXI DMA***: MM2S system Keyhole activated")
            if (status & 0x0000010):
                logging.warning(f"***AXI DMA***: MM2S system Cyclic BD enabled")
            if (status & 0x0001000):
                logging.warning(f"***AXI DMA***: MM2S system Interrupt on complete enabled")
            if (status & 0x0002000):
                logging.warning(f"***AXI DMA***: MM2S system Interrupt on delay enabled")
            if (status & 0x0004000):
                logging.warning(f"***AXI DMA***: MM2S system Interrupt on error enabled")
        elif direction == "s2mm":
            status = self.read_u32(S2MM_CONTROL_REGISTER)
            if (status & 0x00000001):
                logging.warning(f"***AXI DMA***: S2MM system Running")
            if (status & 0x00000004):
                logging.warning(f"***AXI DMA***: S2MM system Reset in progress")
            if (status & 0x0000008):
                logging.warning(f"***AXI DMA***: S2MM system Keyhole activated")
            if (status & 0x0000010):
                logging.warning(f"***AXI DMA***: S2MM system Cyclic BD enabled")
            if (status & 0x0001000):
                logging.warning(f"***AXI DMA***: S2MM system Interrupt on complete enabled")
            if (status & 0x0002000):
                logging.warning(f"***AXI DMA***: S2MM system Interrupt on delay enabled")
            if (status & 0x0004000):
                logging.warning(f"***AXI DMA***: S2MM system Interrupt on error enabled")
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")
        return status

    # Function only when scatter-gather is disabled
    def set_source_address(self, direction, value):
        if direction == "mm2s":
            self.write_u32(MM2S_SOURCE_ADDRESS, value)
        elif direction == "s2mm":
            self.write_u32(S2MM_DEST_ADDRESS, value)
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")

    # Function only when scatter-gather is disabled
    def set_transfer_length(self, direction, value):
        if direction == "mm2s":
            self.write_u32(MM2S_LENGTH, value)
        elif direction == "s2mm":
            self.write_u32(S2MM_LENGTH, value)
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")

    # Function only when scatter-gather is disabled
    def get_transfer_length(self, direction):
        if direction == "mm2s":
            return self.read_u32(MM2S_LENGTH)
        elif direction == "s2mm":
            return self.read_u32(S2MM_LENGTH)
        else:
            raise ValueError("Please use s2mm or mm2s as argument direction!")


class BufferDescriptor():
    def __init__(self, d: udk3api.Device, initial_address: int, final_address: int, memory_initial_address: int,
                 memory_final_address: int, cyclic: bool):
        self.__device = d

        self.__initial_address = initial_address
        self.__final_address = final_address
        self.__number_of_descriptors = int((final_address - initial_address) / 64)
        logging.warning(
            f"***MEM***: Buffer descriptor is from [{hex(initial_address)} to {hex(final_address + 0x40)}[.")

        self.__mem_initial_address = memory_initial_address
        self.__mem_final_address = memory_final_address
        self.__block_buffer_width = int((memory_final_address - memory_initial_address) / self.__number_of_descriptors)
        logging.warning(
            f"***MEM***: Pointing memory from [{hex(memory_initial_address)} to {hex(memory_final_address)}[.")

        # self.mmap = mmap.mmap(fileno, final_address - initial_address, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset = initial_address);
        # self.mmap[:] = bytes(final_address - initial_address)

        self.__is_cyclic = cyclic

    def write_u32(self, offset_address, value):
        self.__device.writeRegister(self.__initial_address + offset_address, value)

    def read_u32(self, offset_address, value):
        return self.__device.readRegister(self.__initial_address + offset_address)

    def get_initial_descriptor_address(self):
        return self.__initial_address

    def get_final_descriptor_address(self):
        if self.__is_cyclic:
            return self.__final_address
        else:
            return self.__final_address - 0x40

    def get_status(self):
        for descriptor in range(self.__number_of_descriptors):
            address_offset = 0x40 * descriptor
            status = self.mmap[0x1C + address_offset: 0x20 + address_offset]
            status = int.from_bytes(status, "little")
            print(descriptor, hex(status))
            # if (status & 0x00000001):

    def set_status_to_zero(self):
        for descriptor in range(self.__number_of_descriptors):
            address_offset = 0x40 * descriptor
            self.mmap[0x1C + address_offset: 0x20 + address_offset] = bytes(4)

    def set_block_descriptor_buffer_width(self, block_width: int):
        self.__block_buffer_width = block_width

    def write_descriptors(self):
        # This is setting the next descriptor address. One points to the next one, offseted by the address
        for descriptor in range(self.__number_of_descriptors):
            address_offset = 0x40 * descriptor
            if descriptor == self.__number_of_descriptors - 1 and self.__is_cyclic:  # End of frame for cyclic is the first one
                # self.mmap[0x00 + address_offset : 0x04 + address_offset] = (self.__initial_address).to_bytes(4, "little")
                self.write_u32(0x00 + address_offset, self.__initial_address)
            else:
                # self.mmap[0x00 + address_offset : 0x04 + address_offset] = (self.__initial_address + (descriptor + 1) * 0x40).to_bytes(4, "little")
                self.write_u32(0x00 + address_offset, self.__initial_address + (descriptor + 1) * 0x40)

        # This is setting the target address in the ddr3 memory
        for descriptor in range(self.__number_of_descriptors):
            address_offset = 0x40 * descriptor
            # self.mmap[0x08 + address_offset : 0x0C + address_offset] = (self.__mem_initial_address + descriptor * self.__block_buffer_width).to_bytes(4, "little")
            self.write_u32(0x08 + address_offset, self.__mem_initial_address + descriptor * self.__block_buffer_width)

        # This is setting the control register. It is the buffer length + SOF/EOF (start of frame / end of frame)
        for descriptor in range(self.__number_of_descriptors):
            address_offset = 0x40 * descriptor
            if descriptor == 0:  # Start of frame
                # self.mmap[0x18 + address_offset : 0x1C + address_offset] = (self.__block_buffer_width | 0x8000000).to_bytes(4, "little")
                self.write_u32(0x18 + address_offset, self.__block_buffer_width | 0x8000000)
            elif descriptor == self.__number_of_descriptors - 1:  # End of frame
                # self.mmap[0x18 + address_offset : 0x1C + address_offset] = (self.__block_buffer_width | 0x4000000).to_bytes(4, "little")
                self.write_u32(0x18 + address_offset, self.__block_buffer_width | 0x4000000)
            else:  # Middle of transfer. SImple the block buffer width
                # self.mmap[0x18 + address_offset : 0x1C + address_offset] = (self.__block_buffer_width).to_bytes(4, "little")
                self.write_u32(0x18 + address_offset, self.__block_buffer_width)

def create_pixel_value(index: int, x: int, y: int, pixel_time: int, pause_time: int, eof: bool, pulsein: bool,
                       pulseout: bool):
    value = ((x & 65535) | (y & 65535) << 16 | (pause_time & 65535) << 32 | (pixel_time & 8191) << 48 | (
                eof & 1) << 61 | (pulsein & 1) << 62 | (pulseout & 1) << 63)
    return value

class ScanDevice():
    def __init__(self, xinitial, yinitial, pixel_time):

        print("UDK3 version : %s" % udk3api.getUdkVersion())
        udk3api.initEx("/home/yvesauad/Documents/NSLumiere/nionswift_plugin/IVG/scan/".encode())
        print("UDK3 version : %s" % udk3api.getUdkVersion())
        udk3api.setLogLevel(0x2)

        for ed in udk3api.enumerate(udk3api.ALL):
            device = ed.open()

        self.__device = device

        self.__device.programFpgaFromBin("/home/yvesauad/FPGA_Projects/efm03_reference_design-v1.2.1/EFM03_exdes/EFM03_exdes.runs/impl_1/efm03_wrapper.bin")


        self.__SG_DMA = list()
        self.__buffer_descriptor = list()

        dma0 = DMA_AXI(device, DmaAxi0)
        bd0 = BufferDescriptor(device, S2MM_BUFFER_INITIAL, S2MM_BUFFER_FINAL, S2MM_MEMORY_INITIAL, S2MM_MEMORY_FINAL, True)
        self.__SG_DMA.append( (dma0, "mm2s") )
        self.__buffer_descriptor.append(bd0)

        dma1 = DMA_AXI(device, DmaAxi1)
        bd1 = BufferDescriptor(device, S2MM_BUFFER_INITIAL2, S2MM_BUFFER_FINAL2, S2MM_MEMORY_INITIAL2, S2MM_MEMORY_FINAL2,
                               True)
        self.__SG_DMA.append( (dma1, "s2mm") )
        self.__buffer_descriptor.append(bd1)

        dma2 = DMA_AXI(device, DmaAxi2)
        bd2 = BufferDescriptor(device, S2MM_BUFFER_INITIAL3, S2MM_BUFFER_FINAL3, S2MM_MEMORY_INITIAL3, S2MM_MEMORY_FINAL3,
                               True)
        self.__SG_DMA.append( (dma2, "s2mm") )
        self.__buffer_descriptor.append(bd2)

        self.__multiBlock = AD5543_AXI(device, AD5543)
        self.__multiBlock.start_transfer(True)
        self.__multiBlock.set_mask('1111')
        self.__multiBlock.set_value(65535)

        self.__magBoard = MagBoard_AXI(device, MAGBOARD)
        self.__magBoard.start_transfer(True)
        self.__magBoard.set_mask('111111')
        self.__magBoard.set_value(16384)
        self.__magBoard.set_switches('000000')

        self.__videoChannels = AD9225_AXI(device, AD9225)
        self.__videoChannels.set_filter_value(0)

        self.generate_scan_list(xinitial, yinitial, pixel_time)
        self.__buffer_descriptor[0].set_block_descriptor_buffer_width(xinitial * xinitial * 8)
        self.initialize_cyclic_buffer()

        self.__x = xinitial
        self.__y = yinitial

        self.__lock = threading.Lock()



    def get_image(self, channel):
        if channel == 0:
            address = S2MM_MEMORY_INITIAL2
        elif channel == 1:
            address = S2MM_MEMORY_INITIAL3
        else:
            address = S2MM_MEMORY_INITIAL3

        with self.__lock:
            image = array.array('B', [0 for i in range(self.__x * self.__y * 2)])
            self.__device.readBlock(address, image)
            image = numpy.frombuffer(image, dtype='uint16').reshape((self.__x, self.__y))
        return image

    def generate_scan_list(self, xmax = 128, ymax = 128, pixel_time = 50, pause_time = 0, serpentine = False, randomize = False):
        indexes = numpy.arange(0, xmax * ymax * 8, 8)

        x_pts = numpy.arange(0, xmax, 1)
        x = x_pts * int(16384 / xmax)

        y_pts = numpy.arange(0, ymax, 1)
        y = y_pts * int(16384 / ymax)

        x_mesh, y_mesh = numpy.meshgrid(x, y)

        if serpentine:
            odd_colums = numpy.arange(1, ymax, 2)
            x_mesh[odd_colums] = numpy.flip(x_mesh[odd_colums])
            pause_time = 0

        x_mesh = x_mesh.flatten()
        y_mesh = y_mesh.flatten()

        if randomize:
            random_array = numpy.arange(0, xmax * ymax, 1)
            numpy.random.shuffle(random_array)
            x_mesh = x_mesh[random_array]
            y_mesh = y_mesh[random_array]

        memory_value = create_pixel_value(indexes, x_mesh, y_mesh, pixel_time, pause_time, False, False, False)
        memory_value[xmax * ymax - 1] = create_pixel_value(xmax * ymax - 1, x_mesh[-1], y_mesh[-1], pixel_time, pause_time, True, False,
                                                           False)
        memory_value_bytes = numpy.frombuffer(memory_value, dtype='uint8')
        #print(memory_value.shape, memory_value_bytes.shape)
        #print(memory_value & 65535)  # X
        #print((memory_value >> 16) & 65535)  # Y
        #print((memory_value >> 61) & 1)  # EOF
        #print((memory_value >> 63) & 1)  # Pulseout

        outb = array.array('B', memory_value_bytes)
        #inb = array.array('B', [0 for i in range(len(memory_value_bytes))])
        self.__device.writeBlock(S2MM_MEMORY_INITIAL, outb)
        #self.__device.readBlock(S2MM_MEMORY_INITIAL, inb)
        #if outb != inb:
        #    raise Exception("Buffers inconsistent.")

    def initialize_cyclic_buffer(self):
        for pair in zip(self.__SG_DMA, self.__buffer_descriptor):
            dma = pair[0][0]
            style = pair[0][1] #"mm2s" or "s2mm"
            bd = pair[1]

            dma.reset_dma(style)
            dma.halt_dma(style)
            dma.enable_interrupt(style)
            dma.enable_cyclic_operation(style)
            bd.write_descriptors()
            dma.set_current_descriptor_address(style, bd.get_initial_descriptor_address())
            dma.start_dma(style)
            dma.set_tail_descriptor_address(style, bd.get_final_descriptor_address())

    def change_scan_parameters(self, x, y, pixel_time_us, flyback, serpentine, randomize):
        with self.__lock:
            self.__x = x
            self.__y = y
            pixel_time_ticks = int(pixel_time_us / 0.01)

            direction = self.__SG_DMA[0][1]
            self.__SG_DMA[0][0].reset_dma(direction)
            self.__SG_DMA[0][0].halt_dma(direction)
            self.__SG_DMA[0][0].enable_interrupt(direction)
            self.__SG_DMA[0][0].enable_cyclic_operation(direction)
            self.__buffer_descriptor[0].set_block_descriptor_buffer_width(x * y * 8)
            self.__buffer_descriptor[0].write_descriptors()

            self.generate_scan_list(x, y, pixel_time_ticks, flyback, serpentine, randomize)

            self.__SG_DMA[0][0].set_current_descriptor_address(direction, self.__buffer_descriptor[0].get_initial_descriptor_address())
            self.__SG_DMA[0][0].start_dma(direction)
            self.__SG_DMA[0][0].set_tail_descriptor_address(direction, self.__buffer_descriptor[0].get_final_descriptor_address())

    def change_video_parameters(self, average):
        with self.__lock:
            self.__videoChannels.set_filter_value(average)
