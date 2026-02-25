
# SDR Implementation of OFDM Channel Estimation and Equalization

## Overview

This repository contains the MATLAB scripts and implementation details for an Orthogonal Frequency Division Multiplexing (OFDM) transceiver. The primary goal of this project is to test and evaluate channel estimation and equalization techniques in real-world wireless environments, addressing challenges like multipath fading and phase distortion.

This project was developed at the Istanbul Technical University Wireless Communication Research Laboratory.

**Authors:** Majd Hamdan, İrem Bahar Şahinkeser 

## System Architecture

The system models a complete OFDM transmission and reception pipeline designed to mitigate channel effects:

**Transmission Pipeline:** An input binary stream is generated and mapped onto a 4-QAM (four-point) constellation.


**Preamble:** The first two symbols serve as an identical training preamble generated from known data to assist the receiver.


**Cyclic Prefix:** A Cyclic Prefix (CP) is prepended to each symbol to prevent Inter-Symbol Interference (ISI).

**Synchronization:** At the receiver, the Symbol Timing Offset (STO) is estimated using a CP correlation method to extract the frame.

**Frequency Correction:** Carrier Frequency Offset (CFO) is calculated and corrected using the Moose algorithm.

**Channel Estimation:** The channel is estimated using the Least Squares (LS) method at specific pilot locations, followed by linear interpolation to obtain the response for data subcarriers.

**Equalization:** Zero-forcing (ZF) equalization is applied to invert the channel effects and recover the transmitted 4-QAM symbols.



## Mathematical Core

The receiver relies on the following theoretical expressions for signal recovery:

**Least Squares (LS) Channel Estimation:** Performed in the frequency domain at the subcarriers containing pilot symbols.
$$\hat{H}_{LS}[k]=\frac{Y_{p}[k]}{X_{p}[k]}$$


**Zero-Forcing (ZF) Equalization:** Reverses the distortion on the data subcarriers using the estimated channel.
$\hat{X}_{ZF}[k]=\frac{Y_{il}[k]}{\hat{H}[k]}$ 



## Hardware and Software Setup

**Software:** Both the simulations and the real-world SDR implementations are executed in MATLAB using the Communications Toolbox.


**Hardware:** The physical test setup utilizes USRP B200 mini Software Defined Radios, with one configured as the transmitter and one as the receiver.


**Frequency:** Over-the-air transmission is configured at a center frequency of 1.2 GHz.



## Repository Files

* `txdemo.m`: Configures the transmitter, generates the random data and training preamble, applies the OFDM modulation, and continuously transmits the waveform over the USRP.
* `rxdemo.m`: Continuously listens for the signal via the receiving USRP, applies STO CP correlation, corrects CFO using the Moose algorithm, and performs LS channel estimation and ZF equalization.

## Results

The system was physically tested indoors within a wireless communication lab to introduce real-world noise and multipath effects.

* The receiver successfully pinpointed the beginning of the frames via CP correlation.


* The Moose algorithm successfully corrected spinning phase noise and frequency offsets.


* By applying LS estimation and ZF equalization, the system maintained stability over multiple frames, successfully resolving the signal into four distinct clusters with a low Bit Error Rate (BER).


