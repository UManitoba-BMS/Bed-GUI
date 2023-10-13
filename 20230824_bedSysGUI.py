"""
Tyson Reimer
University of Manitoba
August 24th, 2023

This is the GUI code for the Python-based bed system control software.


--- Version History ----

v0.1 : 2023-08-24
    Initial version. Capable only of S11 measurements.

v0.2 : 2023-09-05
    Expand capability to include S12, S21, and S22 measurements.

"""

import os
import numpy as np
import tkinter as tk
from tkinter import filedialog
from datetime import datetime
import time
import threading
import serial
import pyvisa as visa
import pickle


###############################################################################
# --- Section 0: Misc. functions ----------------------------------------------

def make_sparam_list():
    """Make the list of S-parameters based on which options set in GUI

    Returns
    -------
    sparams : list
        List containing the S-parameters that will be measured

    """

    sparams = []  # Init list to return

    if meas_s11.get():  # If measuring S11
        sparams.append("S11")

    if meas_s21.get():  # If measuring S21
        sparams.append("S21")

    if meas_s12.get():  # If measuring S12
        sparams.append("S12")

    if meas_s22.get():  # If measuring S22
        sparams.append("S22")

    return sparams


def sparam_log_update():
    """Send update to the logger for S-parameter settings
    """

    log_in_out("S-param settings updated. Measuring:")
    sparam_str = "\t"
    if meas_s11.get():
        sparam_str += "S11 "
    if meas_s21.get():
        sparam_str += "S21 "
    if meas_s12.get():
        sparam_str += "S12 "
    if meas_s22.get():
        sparam_str += "S22"
    log_in_out("\t%s" % sparam_str)

    # If NO S-parameters have been selected
    if (not meas_s11.get() and not meas_s12.get()
            and not meas_s21.get() and not meas_s22.get()):

        # Report caution to logger
        log_in_out("CAUTION: No S-parameter selected. At least one"
                   " must be selected for scanning.")


def save_pickle(var, path):
    """Saves the var to a .pickle file at the path specified

    Parameters
    ----------
    var : object
        A variable that will be saved
    path : str
        The full path of the saved .pickle file
    """

    with open(path, 'wb') as handle:
        pickle.dump(var, handle, protocol=pickle.HIGHEST_PROTOCOL)


def stoppable_sleep(sleep_t):
    """Same as time.sleep(), but allows for stopping

    Parameters
    ----------
    sleep_t : float
        Total sleep time, in [s]
    """

    start_t = time.time()  # The starting time

    # Until the sleep_t has elapsed...
    while (time.time() - start_t) <= sleep_t:

        # Check if the stop_event is set...
        if stop_event.is_set():

            return  # Exit function


###############################################################################
# --- Section 1: Hardware Control functions -----------------------------------


class BedSys:
    """Bed system control class: Controls VNA + rotary stage

    Attributes
    ----------
    sparams :
        List of S-parameters to use in the scan
    vna :
        Instance of VNA() class
    rot_stage:
        Instance of RotaryStage() class
    n_a_pos : int
        Number of antenna positions in the scan
    n_fs : int
        Number of frequency points in the scan
    logger :
        Logger for displaying updates on the GUI

    Methods
    -------
    __init__(self, sparams=None):
        Initialize instance of the BedSys()

    do_pre_scan_checks(self):
        Do pre-scan checks to ensure the rotary stage and VNA are
        connected and communicating with the control PC

    do_scan(self, save_str, dwell_t=1.5):
        Perform a single scan
    """

    def __init__(self, sparams=None):

        self.sparams = sparams

        if sparams is None:  # If no sparams have been set
            self.sparams = ['S11']  # Default so that S11 is set

        for jj in sparams:  # For each sparam, assert it is valid
            assert jj in ['S11', 'S12', 'S21', 'S22'], "Error: invalid sparams"

        self.vna = VNA(sparams=sparams)  # Set up the VNA
        self.rot_stage = RotaryStage()  # Set up the rotary stage

        self.n_a_pos = 72  # Number of antenna positions in scan

        # TODO: Get n_fs from VNA communication
        self.n_fs = 1001  # Number of frequencies in scan

        self.logger = log_in_out  # Get logger

    def do_pre_scan_checks(self):
        """Do pre-scan checks...

        Returns
        -------
        sys_ready : bool
            True if system (rotary stage and VNA) are ready
        """

        rot_pos = self.rot_stage.get_current_pos()  # Get rotary pos

        at_home = rot_pos == 0  # Is the rotary stage at the home pos?

        rot_ready = at_home  # Is the rotary stage ready?

        # TODO: Find way of checking if VNA is ready...
        vna_ready = True

        # Ensure that at least one S-parameter is selected
        sparam_ready = len(self.sparams) >= 1

        # Is system ready?
        sys_ready = rot_ready and vna_ready and sparam_ready

        if not rot_ready:
            log_in_out("Error: Rotary stage not ready.")
        if not vna_ready:
            log_in_out("Error: VNA not ready.")
        if not sparam_ready:
            log_in_out("Error: No S-parameters selected.")

        return sys_ready

    # noinspection PyTypeChecker
    def do_scan(self, save_str, ant_dwell_t=1.5, n_sweeps=1):
        """Perform one scan with the bed system

        Parameters
        ----------
        save_str : str
            Path to where the output will be saved
        ant_dwell_t : float
            Dwell time at each antenna position before a sweep
            is recorded, in [s]
        n_sweeps : int
            Number of sweeps to perform at each antenna position

        Returns
        -------
        fd : array_like
            Frequency domain data
        freqs : array_like
            Frequencies used in the scan
        """

        save_dir = os.path.dirname(save_str)  # Dir where scan is saved
        f_name = os.path.basename(save_str)  # The file name itself

        scan_start_t = time.time()  # Start time of scan

        fd = dict()  # Init dict of frequency domain data
        freqs = 0  # Init freqs

        for s in self.sparams:  # For each sparam

            # Init to be array
            fd[s] = np.zeros([self.n_fs, 2 * n_sweeps * self.n_a_pos],
                             dtype=float)

        self.logger("Starting scan now...")
        for a_pos in range(self.n_a_pos):  # For each position

            self.logger("\tAt position [%3d / %3d]..."
                        % (a_pos + 1, self.n_a_pos))

            stoppable_sleep(ant_dwell_t)  # Wait for mechanical jitter

            self.logger("\t\tWaited %.3f sec. Performing sweeps..."
                        % ant_dwell_t)

            for sweep in range(n_sweeps):  # Do each sweep...

                self.logger("\t\t\tSweep [%2d / %2d]..."
                            % (sweep + 1, n_sweeps))

                # Do the sweep
                sweep_dict, freqs = self.vna.sweep(n_fs=self.n_fs,
                                                   n_sparams=len(self.sparams)
                                                   )
                # Wait some amount between sweeps - note that each
                # sweep takes 266 ms
                stoppable_sleep(0.266)

                for s in self.sparams:  # For each sparam

                    # Record the real + imag parts in adjacent columns
                    fd[s][:, sweep * 2 * self.n_a_pos + (2 * a_pos):
                             sweep * 2 * self.n_a_pos + (2 * a_pos + 2)] = \
                        sweep_dict[s]

            # S11 / S21 sweeps take approx. 133 ms, for
            # 10 kHz IF BW, 2-9 GHz, 1001 pts, 15 dBm
            # S11/S21 and S12/S22 sweeps take approx. 266 ms
            stoppable_sleep(0.5)  # Wait to ensure sweep finishes

            # Unless this is the *last* antenna position...
            if a_pos < (self.n_a_pos - 1):

                self.logger('\t\tMoving to next position...')

                # Take a step to move to the next position
                self.rot_stage.step_CW(step_size=7111)

                # Wait for the mechanical motion to stop
                # NOTE: Measured rotation time of 1.70 s, add 0.2 s
                # for safety
                stoppable_sleep(1.9)

        for s in self.sparams:  # For each S-parameter

            # Save to a file with prefix Snm
            np.savetxt(fname=os.path.join(save_dir, "%s_%s" % (s, f_name)),
                       X=fd[s],
                       delimiter=",")

        # Find time duration of the scan, in [s]
        scan_duration = time.time() - scan_start_t

        self.logger("Scan complete in %d min %d sec. Returning home..."
                    % (scan_duration // 60,
                       scan_duration - (scan_duration // 60) * 60))

        stoppable_sleep(1)  # Wait 1 sec before rotating back to home

        self.rot_stage.go_home()  # Send the rotary stage back home

        # Wait for the stage to rotate all the way home
        # (True time is approx. 33 s, but add 2 s for safety)
        stoppable_sleep(35)

        back_home = self.do_pre_scan_checks()  # Check if back home

        # Report current status to logger
        if back_home:
            self.logger("Returned home successfully. Ready for new scan.")
        else:
            self.logger("ERROR: Failed to return home successfully.")

        return fd, freqs


class VNA:
    """Class to control the Copper Mountain vector network analyzer

    Attributes
    ----------
    sparams :
        List of S-parameters to use in the scan
    vna :
        The VNA as a resource using pyvisa

    Methods
    -------
    __init__(self, sparams=None):
        Initialize instance of the VNA()

    sweep(self, n_fs, s_params):
        Perform a frequency sweep and record the specified S-parameters

    vna_data_out_to_arr(self, vna_out, sparam=True):
        Convert the output from the VNA to an array
    """

    def __init__(self, sparams=None):

        self.sparams = sparams

        if sparams is None:  # If S-parameters not specified
            self.sparams = ['S11']  # Default to S11 only

        for jj in sparams:
            assert jj in ['S11', 'S12', 'S21', 'S22'], "Error: invalid sparams"

        # Get the resource manager for VNA connection
        rm = visa.ResourceManager('@py')

        # Connect to the VNA
        vna = rm.open_resource('TCPIP0::localhost::5025::SOCKET')

        # The VNA ends each line with this
        vna.read_termination = '\n'
        vna.timeout = 10000  # Really long timeout period

        # Set number of trace windows equal to twice number of s-params
        # (because each s-param has real + imag parts)
        vna.write('DISP:WIND:SPL %d' % (2 * len(self.sparams)))
        vna.write('CALC1:PAR:COUN %d' % (2 * len(self.sparams)))

        for ii in range(len(self.sparams)):  # For each sparam

            # Set the first two channels to be the first sparam
            vna.write('CALC1:PAR%d:DEF %s' % (ii * 2 + 1, self.sparams[ii]))
            vna.write('CALC1:PAR%d:DEF %s' % (ii * 2 + 2, self.sparams[ii]))

        for ii in range(len(self.sparams)):  # For each sparam

            # Select first channel for this sparam
            vna.write('CALC1:PAR%d:SEL' % (ii * 2 + 1))
            vna.write('CALC1:FORM REAL')  # Set the channel to meas real

            # Select second channel for this sparam
            vna.write('CALC1:PAR%d:SEL' % (ii * 2 + 2))
            vna.write('CALC1:FORM IMAG')  # Measure imag

        # Scope for saving to include all traces
        vna.write('MMEM:STOR:FDAT:SCOPE ALL')

        # Set trigger source to be internal, to constantly sweep
        vna.write("TRIG:SOUR INT\n")

        # End the VNA setup with this command to ensure all are complete
        vna.query("*OPC?\n")

        # Store the copper
        self.vna = vna

    def sweep(self, n_fs, n_sparams):
        """Perform a sweep and measure result

        Parameters
        ----------
        n_fs : int
            Number of frequencies in the scan
        n_sparams : int
            Number of s-parameters in the scan

        Returns
        -------
        fd_sig : array_like
            Complex-valued frequency domain signal measured in the
            sweep
        """

        self.vna.write("TRIG:SING\n")  # Trigger a single sweep
        self.vna.query("*OPC?\n")  # Wait for sweep to complete

        fd_data = dict()

        for ii in range(len(self.sparams)):  # For each sparam

            # Select the trace corresponding to real part of this sparam
            self.vna.write('CALC1:PAR%d:SEL' % (ii * 2 + 1))

            # Read the real part of the S-parameter
            re_data = self.vna_data_out_to_arr(
                self.vna.query("CALC1:DATA:FDAT?")
            )

            # Select the trace corresponding to imag part of this sparam
            self.vna.write('CALC1:PAR%d:SEL' % (ii * 2 + 2))

            # Read the imag part of the S-parameter
            im_data = self.vna_data_out_to_arr(
                self.vna.query("CALC1:DATA:FDAT?")
            )

            # Store result in array
            fd_data[self.sparams[ii]] = np.vstack((re_data, im_data)).T

        # Record the frequencies used in the measurement
        freqs = self.vna_data_out_to_arr(self.vna.query("SENS1:FREQ:DATA?"),
                                         sparam=False)

        return fd_data, freqs

    def vna_data_out_to_arr(self, vna_out, sparam=True):
        """Convert the VNA output to array-type

        Parameters
        ----------
        vna_out :
            The output of the VNA
        sparam : bool
            True if vna_out was an S-parameter measurement

        Returns
        -------
        arr_data :
            The output of the VNA as an array
        """

        if sparam:
            arr_data = np.array([float(ii) for ii in vna_out.split(",")[::2]])
        else:
            arr_data = np.array([float(ii) for ii in vna_out.split(",")])
        return arr_data


class RotaryStage:
    """Class for controlling the rotary stage in the bed system

    Attributes
    ----------
    rot :
        serial.Serial() instance for communication with the rotary stage

    Methods
    -------
    __init__(self):
        Initialize instance of the RotaryStage()

    stop_moving(self):
        Stop the rotary stage from moving (i.e., will STOP movement)

    step_CW(self, step_size=7111):
        Step by amount step_size in the clockwise direction

    step_CCW(self, step_size=7111):
        Step by amount step_size in the counter-clockwise direction

    read_fr_rot(self):
        Read the response from the rotary stage

    get_current_pos(self):
        Get the current position of the rotary stage

    go_home(self):
        Send the rotary stage back to the 'home' position
    """

    def __init__(self):

        # Get the rotary stage connection
        rot = serial.Serial(port='COM3', baudrate=19200,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS)

        if rot.isOpen():  # If the rotary is already open
            rot.close()  # Then close
        rot.open()  # And re-open...

        self.rot = rot  # The rotary stage

    def stop_moving(self):
        """Stop the rotary stage from moving (rotating)
        """

        self.rot.write("SPX,0,0\r".encode())

    def step_CW(self, step_size=7111):
        """Take a step in the positive rotation (CW) direction

        Parameters
        ----------
        step_size : int
            Number of microsteps to perform, standard is 7111
        """

        step_str = "+X,%d,0\r" % step_size  # Str for stage command
        self.rot.write(step_str.encode())  # Send command to stage

    def step_CCW(self, step_size=7111):
        """Take a step in the negative rotation (CCW) direction

        Parameters
        ----------
        step_size : int
            Number of microsteps to perform, standard is 7111
        """

        step_str = "-X,%d,0\r" % step_size  # Str for stage command
        self.rot.write(step_str.encode())  # Send command to stage

    def read_fr_rot(self):
        """Read from the rotary stage

        Returns
        -------
        out : str
            The response from the rotary stage
        """

        out = ''  # Init str for return

        while self.rot.inWaiting() > 0:  # While it has a message waiting
            x = self.rot.read()  # Read from the rotary stage
            out += x.decode()  # Store this part of the message

        # out = self.rot.readline()
        return out

    def get_current_pos(self):
        """Get the current position of the rotary stage

        Returns
        -------
        cur_pos : int
            The current position of the rotary stage
        """

        # Reset the buffer in case a lingering output remains
        self.rot.reset_input_buffer()
        self.rot.reset_output_buffer()  # Flush the output buffer

        get_current_pos = "?X,0,0\r"  # Str for getting current x pos
        self.rot.write(get_current_pos.encode())  # Ask for x pos

        time.sleep(1)  # Wait a moment for the response

        # Read the response from the rotary stage
        out = self.read_fr_rot()

        if "/r" in out:
            cur_pos = int(out.split(',')[1][:-1])
        else:
            cur_pos = int(out.split(',')[1])
        return cur_pos

    def go_home(self):
        """Send to home by first rotating near home then 'send home'
        """

        cur_pos = self.get_current_pos()  # Get current rotary position

        stoppable_sleep(1.0) # Wait for the response

        # Move back to home
        self.step_CCW(step_size=cur_pos)


###############################################################################


class StoppableThread(threading.Thread):
    """A thread that is safely stoppable

    Attributes
    ----------
    stop_event :
        Event to signal the STOPPING of the thread
    target :
        The target function to be executed with the thread
    args :
        Any args for the target function

    Methods
    -------
    __init__(self):
        Initialize instance of the StoppableThread()

    run(self):
        Run the thread in stoppable-mode

    """

    def __init__(self, stop_event, target, args=()):
        """Init instance of StoppableThread()

        Parameters
        ----------
        stop_event :
            Event that signals STOPPING of the thread
        target :
            The target function to be executed by the thread
        args :
            Any args for the target function
        """

        super().__init__()  # Init

        # Set instance attributes
        self.stop_event = stop_event
        self.target = target
        self.args = args

    def run(self):
        """Run the thread in stoppable-mode
        """

        # If the stop_event has not been triggered AND if the
        # target function is being executed
        if not self.stop_event.is_set() and self.target:
            self.target(*self.args)


def stop_all_threads():
    """Stops all stoppable threads
    """

    # Set the stop_event (therefore triggering STOP on all
    # stoppable threads)
    stop_event.set()


def log_in_out(out_str):
    """Report to the GUI logger

    Parameters
    ----------
    out_str : str
        String to report in the logger
    """

    # Get the current time-stamp
    current_time = datetime.now().strftime("[%Y-%m-%d %H:%M:%S]")

    # Write to the logger display in the GUI
    out_txt.insert(tk.END,
                   f"{current_time}:\t"
                   "%s\n" % out_str)

    # Scroll to the bottom of the logger display
    out_txt.see(tk.END)


def do_one_scan(save_str, dwell_t, n_sweeps=1):
    """Do one scan with the bed system

    Parameters
    ----------
    save_str : str
        The path to the .txt file that will be saved
    dwell_t : float
        Amount of time, in [sec], to wait at each position before
        performing a VNA sweep, [s]
    n_sweeps : int
        Number of sweeps to perform at each antenna position
    """

    # Tell the bed system to do a scan
    _, freqs = bed.do_scan(save_str=save_str,
                           ant_dwell_t=dwell_t,
                           n_sweeps=n_sweeps)

    # Save the frequencies to the dir
    np.savetxt(fname=os.path.join(os.path.dirname(save_str), "scan_fs.txt"),
               X=freqs,
               delimiter=","
               )

    # Allow the do_scan_button to be pushed again, *after* the scan
    # has been completed
    do_scan_button.config(state=tk.NORMAL)


def do_n_scans(save_dir, n_scans, dwell_t, n_sweeps=1):
    """Do n scans instead of just one scan

    Parameters
    ----------
    save_dir : str
        Path to dir where the .txt files and .pickle file will be
        saved
    n_scans : int
        Number of scans to perform
    dwell_t : float
        Amount of time, in [sec], to wait at each position before
        performing a VNA sweep, [s]
    n_sweeps : int
        Number of sweeps to do at each antenna position
    """

    fd = dict()  # Init dict of frequency domain data
    fs = 0  # Init fs to remove warning

    for s in bed.sparams:  # For each sparam

        # Init to be array
        fd[s] = np.zeros([n_scans, bed.n_fs, n_sweeps * 2 * bed.n_a_pos],
                         dtype=float)

    for ii in range(n_scans):  # For each scan...

        log_in_out("**Performing scan [%2d / %2d]..."
                   % (ii + 1, n_scans))

        # Take a scan
        fd_dict, fs = bed.do_scan(
            save_str=os.path.join(save_dir, 'exp%d.txt' % ii),
            ant_dwell_t=dwell_t,
            n_sweeps=n_sweeps,
        )

        for s in bed.sparams:  # For each S-parameter
            fd[s][ii, :, :] = fd_dict[s]  # Store frequency domain data

    # Save the frequencies to the dir
    np.savetxt(fname=os.path.join(save_dir, "scan_fs.txt"),
               X=fs,
               delimiter=","
               )

    for s in bed.sparams:  # For each S-parameter

        this_s = fd[s]  # The set of data for this S-parameter

        # Init arr for saving complex .pickle file
        fd_to_save = np.zeros([np.size(this_s, axis=0),
                               np.size(this_s, axis=1),
                               np.size(this_s, axis=2) // 2],
                              dtype=complex)

        # For each antenna position
        for aa in range(np.size(this_s, axis=2) // 2):

            # Get the complex-valued S-parameters
            fd_to_save[:, :, aa] = (this_s[:, :, 2 * aa]
                                    + 1j * this_s[:, :, 2 * aa + 1])

        # Save to a .pickle file
        save_pickle(fd_to_save, os.path.join(save_dir, '%s.pickle' % s))

    # Reset the do_scan button so that it can again be pushed
    do_scan_button.config(state=tk.NORMAL)


###############################################################################


def create_bed_sys():
    """Connect to bed system by making instance of class
    """

    global bed  # Ensure the bed is used throughout this script

    sparams = make_sparam_list()  # Get the s-param list

    try:  # Try to connect to the bed system

        bed = BedSys(sparams=sparams)  # Connect to the bed system

        log_in_out("Bed system connected.")

        # Update the Mandatory Steps counter
        completed_steps.set(completed_steps.get() + 1)

        # Update the label and the logger
        do_scan_label.config(text="[%d / %d] Mandatory Steps Completed"
                                  % (completed_steps.get(),
                                     tot_steps.get()))
        log_in_out("[%d / %d] Mandatory steps completed."
                   % (completed_steps.get(), tot_steps.get()))

        # Enable the pre-scan auto-check button
        pre_scan_button.config(state=tk.NORMAL)

        log_in_out("NOTE: S-parameter selection disabled after bed system "
                   "connection. Please close & reopen GUI to change"
                   " S-parameter selection.")
        # Disable the S-parameter selection after connecting to bed
        s11_button.config(state=tk.DISABLED)
        s21_button.config(state=tk.DISABLED)
        s12_button.config(state=tk.DISABLED)
        s22_button.config(state=tk.DISABLED)

    except Exception as e:  # If something went wrong, report it

        log_in_out("ERROR: Could not connect to bed system.")
        log_in_out(f"Error: {e}")


def do_prescan_checks():
    """Do automated pre-scan checks on if the rotary stage + VNA ready
    """

    # Check the VNA and rotary stage
    bed_ready = bed.do_pre_scan_checks()

    if bed_ready:  # If the bed system is ready for a scan

        # Report this to the logger
        log_in_out("Pre-scan checks completed successfully. Please"
                   " complete manual system inspection.")

        # Enable the manual inspection button
        man_insp_button.config(state=tk.NORMAL)

        # Update Mandatory Steps counter
        completed_steps.set(completed_steps.get() + 1)
        do_scan_label.config(text="[%d / %d] Mandatory Steps Completed"
                                  % (completed_steps.get(),
                                     tot_steps.get()))
        log_in_out("[%d / %d] Mandatory steps completed."
                   % (completed_steps.get(), tot_steps.get()))

    else:  # If the bed system is not ready
        log_in_out("ERROR: Pre-scan checks failed.")


def do_scan():
    """Perform a scan
    """

    # Ensure threads are stoppable
    global stop_event

    # Create stop event for stopping threads
    stop_event = threading.Event()

    new_thread = False  # Flag for executing new threads

    if n_scans.get() == 1:  # If doing 1 scan

        # Get the user to select the save_str used to save the .txt
        # file generated in the scan
        save_str = filedialog.asksaveasfilename(
            title="Save file",
            initialdir=fdiag_default_dir.get(),
            filetypes=[("Text Files", "*.txt")],
            defaultextension=".txt",
        )

        if save_str == "":  # If save_str is empty
            log_in_out("File dialog request aborted.")

        else:  # If save_str was set by user

            # Find the dir to save the file
            save_dir = os.path.dirname(save_str)
            fdiag_default_dir.set(save_dir)

            log_in_out("Will save file as: %s" % save_str)

            # Create a stoppable thread for dong 1 scan
            new_thread = StoppableThread(
                target=do_one_scan,
                args=(save_str, dwell_t.get(), n_sweeps.get()),
                stop_event=stop_event
            )

    else:  # If doing more than 1 scan

        # Ask the user to select a directory only (the fnames will be
        # automatically generated as exp0.txt, exp1.txt, etc.)
        save_dir_str = filedialog.askdirectory(
            initialdir=fdiag_default_dir.get(),
        )

        if save_dir_str == "":  # If the user didn't choose a dir
            log_in_out("File dialog request aborted.")

        else:  # If the user did choose a dir

            # Set this to be the default save dir while GUI is open
            fdiag_default_dir.set(save_dir_str)

            log_in_out("Will save files to dir: %s" % save_dir_str)

            # Create a stoppable thread for doing N scans
            new_thread = StoppableThread(
                target=do_n_scans,
                args=(fdiag_default_dir.get(),
                      n_scans.get(),
                      dwell_t.get(),
                      n_sweeps.get(),
                      ),
                stop_event=stop_event
            )

    if new_thread:  # If ready to scan...

        log_in_out("Starting scanning procedure, measuring S-parameters:")

        for ii in bed.sparams:  # Report all S-params
            log_in_out("\t%s" % ii)

        # Disable the scan button so that it can't be pressed twice by accident
        do_scan_button.config(state=tk.DISABLED)

        new_thread.start()  # Start the thread to perform the scan


def stop_temp():
    """Stop *everything* by killing all threads
    """

    # Report to logger
    log_in_out("EMERGENCY STOP ACTIVATED.")

    try:  # Try to stop the system

        bed.rot_stage.stop_moving()  # Stop the stage from moving
        bed.rot_stage.rot.close()  # Close connection to rotary stage
        stop_all_threads()  # Stop all active threads

        # Report to logger
        log_in_out("System successfully stopped.")
        log_in_out("GUI must be closed and re-launched for further use.")
        log_in_out("Please manually return system to HOME and inspect"
                   " before further use.")

        # Disable the do_scan_button to indicate to user that
        # GUI needs to be re-launched
        do_scan_button.config(state=tk.DISABLED)

    except:  # If the system couldn't be stopped for some reason
        log_in_out("ERROR: System stop unsuccessful.")


def set_dwell_t():
    """Gets the user-entered dwell_t
    """

    try:  # Try to get the dwell_t value...

        val = float(dwell_t_entry.get())

        dwell_t.set(val)  # Set the value

        # Update the status indicator
        dwell_t_status.config(text=f"dwell_t set to: {val} sec")

        log_in_out("dwell_t set to: %.3f sec" % val)

    except ValueError:

        log_in_out("Invalid input for dwell_t. Please enter a number.")


def set_n_scans():
    """Get the user-entered n_scans
    """

    try:  # Try to get the dwell_t value...

        val = int(n_scans_entry.get())

        n_scans.set(val)  # Set the value

        # Update status indicator...
        n_scans_status.config(text=f"n_scans set to: {val}")

        log_in_out("n_scans set to: %d" % val)

    except ValueError:

        log_in_out("Invalid input for n_scans. Please enter an integer.")


def set_n_sweeps():
    """Get the user-entered n_scans
    """

    try:  # Try to get the dwell_t value...

        val = int(n_sweep_entry.get())

        n_sweeps.set(val)  # Set the value

        # Update status indicator...
        n_sweeps_status.config(text=f"n_sweeps set to: {val}")

        log_in_out("n_sweeps set to: %d" % val)

    except ValueError:

        log_in_out("Invalid input for n_sweeps. Please enter an integer.")


def confirm_manual_inspect():
    """After user clicks on button, allow scans to be performed
    """

    # Enable the do_scan_button, so that scans can  be performed
    do_scan_button.config(state=tk.NORMAL)

    log_in_out("Manual inspection CONFIRMED.")

    # Update Mandatory Steps counter
    completed_steps.set(completed_steps.get() + 1)
    log_in_out("Completed [%d / %d] mandatory steps."
               % (completed_steps.get(), tot_steps.get()))
    do_scan_label.config(text="[%d / %d] Mandatory Steps Completed"
                              % (completed_steps.get(), tot_steps.get()))


###############################################################################


if __name__ == "__main__":

    # ------ Create main window -----------------------------------------------
    root_tk = tk.Tk()  # Create main window

    root_tk.title("Test GUI")  # Give it a name

    gui_width = 1600  # Define GUI width, in pixels
    gui_height = 900  # Define GUI height, in pixels

    # TODO: Uncomment to save .ico
    # root_tk.iconbitmap(default="C:/Users/admin/Desktop/gui_symb.ico")

    root_tk.configure(bg="#fff0fc")

    # Set the width/height
    root_tk.geometry(f"{gui_width}x{gui_height}")

    # Lift window and give it focus
    root_tk.lift()
    root_tk.focus_force()

    # Add title label
    title_label = tk.Label(root_tk,
                           text="Bed System Control Software"
                                " (Experimental Use Only) v0.2",
                           font=("Times New Roman", 24),
                           bg="#ffcff4",
                           justify=tk.CENTER,
                           bd=2,
                           relief="solid"
                           )
    title_label.pack(pady=10)

    # Create str variable for storing the default dir for the fdiag
    fdiag_default_dir = tk.StringVar()
    fdiag_default_dir.set(os.path.join(os.path.expanduser("~"), "Documents/"))

    # ------ Create button for setting S-params--------------------------------

    # Init bool vars for indicating which S-parameters should be saved
    meas_s11 = tk.BooleanVar(value=True)
    meas_s21 = tk.BooleanVar(value=False)
    meas_s12 = tk.BooleanVar(value=False)
    meas_s22 = tk.BooleanVar(value=False)

    # Label for which s-params to measure
    sparam_label = tk.Label(root_tk,
                            text="Pre-Connect Step: Select S-params to "
                                 "Measure:",
                            font=("Times New Roman", 20),
                            bg="#ffcff4",
                            justify=tk.LEFT,
                            bd=2,
                            relief="solid"
                            )
    sparam_label.place(x=50, y=60)

    # Make CheckButton to indicate if S11 should be measured
    s11_button = tk.Checkbutton(root_tk,
                                text="S11",
                                bg="#e1e1e1",
                                justify=tk.CENTER,
                                bd=2,
                                relief="raised",
                                font=("Times New Roman", 20),
                                variable=meas_s11,
                                command=sparam_log_update,
                                )
    s11_button.place(x=50, y=100)

    # Make CheckButton to indicate if S21 should be measured
    s21_button = tk.Checkbutton(root_tk,
                                text="S21",
                                bg="#e1e1e1",
                                justify=tk.CENTER,
                                bd=2,
                                relief="raised",
                                font=("Times New Roman", 20),
                                variable=meas_s21,
                                command=sparam_log_update,
                                )
    s21_button.place(x=150, y=100)

    # Make CheckButton to indicate if S12 should be measured
    s12_button = tk.Checkbutton(root_tk,
                                text="S12",
                                bg="#e1e1e1",
                                justify=tk.CENTER,
                                bd=2,
                                relief="raised",
                                font=("Times New Roman", 20),
                                variable=meas_s12,
                                command=sparam_log_update,
                                )
    s12_button.place(x=250, y=100)

    # Make CheckButton to indicate if S22 should be measured
    s22_button = tk.Checkbutton(root_tk,
                                text="S22",
                                bg="#e1e1e1",
                                justify=tk.CENTER,
                                bd=2,
                                relief="raised",
                                font=("Times New Roman", 20),
                                variable=meas_s22,
                                command=sparam_log_update,
                                )
    s22_button.place(x=350, y=100)

    # ------ Create button for connecting -------------------------------------

    connect_label = tk.Label(root_tk,
                             text="Mandatory Step (1):",
                             font=("Times New Roman", 20),
                             bg="#ffcff4",
                             justify=tk.LEFT,
                             bd=2,
                             relief="solid"
                             )
    connect_label.place(x=50, y=170)

    # Make a button for now
    connect_button = tk.Button(root_tk, text="Connect to Bed System",
                               command=create_bed_sys,
                               bg="#e1e1e1",
                               justify=tk.CENTER,
                               bd=2,
                               relief="raised",
                               font=("Times New Roman", 20))
    connect_button.place(x=50, y=210)

    # ------ Create button for pre-scan checklist -----------------------------

    auto_check_label = tk.Label(root_tk,
                                text="Mandatory Step (2):",
                                font=("Times New Roman", 20),
                                bg="#ffcff4",
                                justify=tk.LEFT,
                                bd=2,
                                relief="solid"
                                )
    auto_check_label.place(x=50, y=280)

    # Make a button to execute pre-scan checks
    pre_scan_button = tk.Button(root_tk, text="Do Automated Pre-Scan Checks",
                                # command=bed.do_pre_scan_checks,
                                command=do_prescan_checks,
                                bg="#e1e1e1",
                                justify=tk.CENTER,
                                bd=2,
                                relief="raised",
                                font=("Times New Roman", 20),
                                state=tk.DISABLED)
    pre_scan_button.place(x=50, y=320)

    # ------ Create button for manual rotary stage inspection -----------------

    man_check_label = tk.Label(root_tk,
                               text="Mandatory Step (3):",
                               font=("Times New Roman", 20),
                               bg="#ffcff4",
                               justify=tk.LEFT,
                               bd=2,
                               relief="solid"
                               )
    man_check_label.place(x=50, y=390)

    man_insp_button = tk.Button(root_tk,
                                text="Click to Confirm Manual Visual "
                                     "Inspection"
                                     " of the\nRotary Stage: Confirm at "
                                     "Position x=0",
                                bg="#e1e1e1",
                                justify=tk.CENTER,
                                bd=2,
                                relief="raised",
                                font=("Times New Roman", 20),
                                command=confirm_manual_inspect,
                                state=tk.DISABLED
                                )

    man_insp_button.place(x=50, y=430)

    # ------ Create text input window for dwell_t  ----------------------------

    opt_frame = tk.Frame(root_tk,
                         bg="#fff0fc",
                         highlightbackground="black",
                         highlightthickness=2)
    opt_frame.place(x=30, y=520, width=650, height=280)

    opt_frame_label = tk.Label(root_tk,
                               text="Optional Steps",
                               bd=2,
                               bg="#ffcff4",
                               justify=tk.LEFT,
                               relief="solid",
                               font=("Times New Roman", 20),
                               )
    opt_frame_label.place(x=50, y=535)

    dwell_t_entry = tk.Entry(root_tk,
                             font=("Times New Roman", 16),
                             bd=2,
                             justify=tk.RIGHT)
    dwell_t_entry.place(x=355, y=580)

    dwell_t_button = tk.Button(root_tk,
                               text="Set Antenna Dwell Time (s):",
                               command=set_dwell_t,
                               bg="#e1e1e1",
                               justify=tk.CENTER,
                               bd=2,
                               font=("Times New Roman", 16),
                               relief="raised",
                               )
    dwell_t_button.place(x=50, y=580)

    # Create DoubleVar to store the value of dwell_t
    dwell_t = tk.DoubleVar()

    # Initialize dwell_t value to 2.5 s
    dwell_t.set(2.5)

    dwell_t_status = tk.Label(root_tk,
                              text="dwell_t set to %.3f sec (default)"
                                   % dwell_t.get(),
                              bd=2,
                              bg="#ffcff4",
                              justify=tk.CENTER,
                              font=("Times New Roman", 16),
                              relief="solid",
                              )
    dwell_t_status.place(x=360, y=610)

    # ------ Create text input window for doing N scans  ----------------------

    n_scans_entry = tk.Entry(root_tk,
                             font=("Times New Roman", 16),
                             bd=2,
                             justify=tk.RIGHT)
    n_scans_entry.place(x=355, y=650)

    n_scans_button = tk.Button(root_tk,
                               text="Enter Number of Scans to Do:",
                               command=set_n_scans,
                               bg="#e1e1e1",
                               justify=tk.CENTER,
                               bd=2,
                               font=("Times New Roman", 16),
                               relief="raised",
                               )
    n_scans_button.place(x=50, y=650)

    # Create integer variable for n_scans
    n_scans = tk.IntVar()
    n_scans.set(1)  # Initialize n_scans to 1

    n_scans_status = tk.Label(root_tk,
                              text="n_scans set to %d (default)"
                                   % n_scans.get(),
                              bd=2,
                              bg="#ffcff4",
                              justify=tk.CENTER,
                              font=("Times New Roman", 16),
                              relief="solid",
                              )

    n_scans_status.place(x=360, y=680)

    # ------ Create button for setting n_sweeps -------------------------------

    n_sweep_entry = tk.Entry(root_tk,
                             font=("Times New Roman", 16),
                             bd=2,
                             justify=tk.RIGHT)
    n_sweep_entry.place(x=355, y=730)

    n_sweep_button = tk.Button(root_tk,
                               text="Set Number of Sweeps\nper Position:",
                               command=set_n_sweeps,
                               bg="#e1e1e1",
                               justify=tk.CENTER,
                               bd=2,
                               font=("Times New Roman", 16),
                               relief="raised",
                               )
    n_sweep_button.place(x=50, y=730)

    # Create integer variable for n_sweeps
    n_sweeps = tk.IntVar()
    n_sweeps.set(1)  # Initialize n_sweeps to 1

    n_sweeps_status = tk.Label(root_tk,
                               text="n_sweeps set to %d (default)"
                                    % n_sweeps.get(),
                               bd=2,
                               bg="#ffcff4",
                               justify=tk.CENTER,
                               font=("Times New Roman", 16),
                               relief="solid",
                               )

    n_sweeps_status.place(x=360, y=760)

    # ------ Create text output window for logging ----------------------------

    # Make label for the output text box
    out_txt_label = tk.Label(root_tk,
                             text="Logger Output:",
                             font=("Times New Roman", 16),
                             bg="#ffcff4",
                             justify=tk.LEFT,
                             bd=2,
                             relief="solid")
    out_txt_label.place(x=1100, y=95)

    # Add a text box for reporting logging outputs...
    out_txt = tk.Text(root_tk,
                      height=35,
                      width=110,
                      bd=2,
                      relief="solid")
    out_txt.place(x=710, y=140)

    # TODO: Add scrollbar for out_txt widget

    log_in_out("Welcome to the *EXPERIMENTAL* bed system GUI.")
    log_in_out("Please complete the necessary pre-scan checks as follows:")
    log_in_out(("\t(0) Select S-parameters to measure (optional)"))
    log_in_out("\t(1) Connect to Bed System")
    log_in_out("\t(2) Do Automated Pre-Scan Checks")
    log_in_out("\t(3) Do Manual Visual Inspection of the Rotary Stage")
    log_in_out("The system will be ready to scan only after completing these "
               "pre-scan checks. (The 'Perform Scan(s)' button will be "
               "enabled after completing these checks.)")

    # ------ Create button for emergency STOP ---------------------------------

    # Make a button to execute pre-scan checks
    stop_button = tk.Button(root_tk,
                            text="EMERGENCY STOP",
                            command=stop_temp,
                            bg="#ff0000",
                            justify=tk.CENTER,
                            bd=5,
                            fg='#000000',
                            relief="raised",
                            font=("Times New Roman", 40),
                            )
    stop_button.place(x=1050, y=750)

    # ------ Create button for performing a scan ------------------------------

    completed_steps = tk.IntVar()
    completed_steps.set(0)
    tot_steps = tk.IntVar()
    tot_steps.set(3)

    do_scan_label = tk.Label(root_tk,
                             text="[%d / %d] Mandatory Steps Completed"
                             % (completed_steps.get(), tot_steps.get()),
                             font=("Times New Roman", 20),
                             bg="#ffcff4",
                             justify=tk.LEFT,
                             bd=2,
                             relief="solid"
                             )
    do_scan_label.place(x=50, y=835)

    # Make a button to perform scan(s)
    do_scan_button = tk.Button(root_tk, text="Perform Scan(s)",
                               command=do_scan,
                               bg="#e1e1e1",
                               justify=tk.CENTER,
                               bd=2,
                               relief="raised",
                               font=("Times New Roman", 30),
                               state=tk.DISABLED)
    do_scan_button.place(x=470, y=810)

    # ------ Launch the GUI ---------------------------------------------------

    root_tk.mainloop()  # Start the GUI
