/* Cosby is TI99/4a data cassette interface software modem */
/* Copyright (C) 2012 Nicholas Nassar */

/* -------------------------------------------------------------------
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ------------------------------------------------------------------- */
/*
Cosby is a program for Linux that will encode and decode the audio
signals sent to and from the data cassette port of a TI99/4a
computer. It requires a very simple to construct cable to connect the
DB9 port of the TI99/4a to audio input and output cables.See the
README file for details on how to make the cable.

It's usable as a simple software modem without the TI99/4a. The signal
is encoded using frequency shift keying (FSK), which is a fancy way of
saying when it's sending a "0" the signal is at a certain frequency
and when it's sending a "1", it's is at a different frequency.  The
"play" function creates an audio output signal corresponding to a
file, and the "record" function turns an audio signal back into a
file.

The TI99/4a sends half a wave at 689Hz for "0" and a full wave at
1378Hz for "1" (At least, that's roughly the frequencies mine uses).

The data starts with lots of "0"s followed by eight "1"s.

To encode, cosby creates audio signals with the appropriate frequencies
using FM synthesis with the Fast Fourier Transform (FFT).

To decode, cosby compares the level of the "0" frequency (689Hz) and
the "1" frequency (1378Hz), which is isolates with the FFT. When the
"0" frequency signal becomes stronger, it registers a "0". When the
"1" becomes stronger, it registers a "1".  If the same signal is
strong, it repeats the last "0" or "1".

Mathematicians and engineers tend to make the FFT sound more
complicated than it is. The fourier transform divides a signal up into
the frequencies that it's composed of, its harmonics. The meaning of
"harmonics" here is exactly the same as harmonics in music-
frequencies that are multiples of the base frequency.

So, given an audio signal of a certain length, the FFT will return the
strength of the following components: the constant part of the signal
(engineers call this the DC offset), the signal consisting of a single
wave of that length, the signal twice that frequency (half the
wavelength), the signal three times that frequency (one-third the
wavelength), and so on.

It also gives the phase of the signal at that frequency. The phase is
how much the waveform is shifted over. It does this by giving two
numbers: one value for the amount that starts at its peak and a
second for the amount that starts at zero. This is what the
mathematicians are referring to when they say "real" and "imaginary"
parts.

To decode the signal, cosby performs the FFT over a 689Hz section of
audio. The 0th position in the resulting array is the offset. The 1st
position is the amount at 689Hz - the "0" signal. The 2nd position is
the amount at 1378Hz - the "1" signal.

To encode the signal, it performs the process in reverse. It sets the
1st ("0") and 2nd ("1") positions in a frequency representation of the
array, then it performs a reverse FFT to get the audio signal.

*/

/* =======================================================
          (Somewhat) User Configurable Definitions
   ======================================================= */

/* Change this to OUTPUT_DEBUG to print extra debugging info.
   Not particuarly useful at this point. */
#define DEFAULT_OUTPUT_LEVEL OUTPUT_NORMAL

/* This is the sample rate cosby uses in .wav files it creates and
   when it opens the audio device. This is higher than it needs to be.
   It's the sample rate most likely to be available on your Linux box.
   Changing this to a lower rate is the easiest, best way to make
   cosby require less CPU power.
*/
#define DEFAULT_SAMPLE_RATE 44100

/* This is the frequency in Hz of the "0" symbol. The frequency of the
"1" symbol is exactly twice the frequency of the "0" symbol */
#define ZERO_FREQ 1378

/* The number of seconds to wait before deciding there's no signal */
#define MAX_WAIT 30

/* The signal power can be this many times weaker than it's strength when it
   was framed before we decide the transmission is complete. You might make
   this larger if cosby thinks it's finished before it's really done or smaller
   if it never finishes. */
#define SIGNAL_POWER_RANGE 16.0

/* The TI99/4a uses its remote control functionality to start and stop
   a real data cassette deck. Cosby had to figure out when the signal
   gets weak to decide when to stop. To do this, it keeps track of the
   average power in the frequncy ranges we care about. This is the
   size of the range it averages in symbols (half a wave at the low
   frequency). 

   You probably won't need to touch it. */
#define POWER_SQ_TOTALS_SIZE 2.0

/* You'd think this would be called "default." There is something called
   "default," but for some reason this weird string is what you're supposed
   to use. */
#define ALSA_AUDIO_DEVICE "plughw:0,0"

/* The number of samples to keep in memory when reading from a file.
   Cosby reads in data in blocks of half this many samples. */
#define AUDIO_BUFFER_SIZE 4096


/* =======================================================
                        INCLUDES
   ======================================================= */

/* Standard C library headers */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <alloca.h>
#include <stdarg.h>

/* ALSA is used for audio input and output
   Read about it here http://www.alsa-project.org/ */
#include <alsa/asoundlib.h>

/* FFTW3 is the library I use for Fast Fourier Transforms 
   Get it here: http://www.fftw.org/ */
#include <fftw3.h>

/* libsndfile is used to read and write WAV files
   http://www.mega-nerd.com/libsndfile/ */
#include <sndfile.h>


/* Wavelength in samples */
#define DEFAULT_WAVELENGTH ((size_t)(DEFAULT_SAMPLE_RATE/(double)ZERO_FREQ+0.5)) 
#define DEFAULT_SYMBOL_LENGTH ((size_t)((DEFAULT_SAMPLE_RATE/(double)ZERO_FREQ)/2.0+0.5))

/* Nobody said there would be math involved. Screw you guys. I'm going
   home. */
#define PI 4.0*atan(1)

#define OUTPUT_NORMAL 1
#define OUTPUT_DEBUG  2
#define OUTPUT_STDERR 4

/* This program works on chips that arrange binary digits from biggest
   to littlest as well as chips that arrange bytes from littlest to
   biggest. This bit of code figures out which type of machine it's
   running on. */
const int endian_test = 1;
#define is_bigendian() ( (*(char*)&endian_test) == 0 )

/* =======================================================
                        Globals
   ======================================================= */


/* Chill out dude. A few tasteful globals in a single-file program
   never killed anyone.

   We're trained to think globals are evil. Java doesn't even allow them.
   But, they make for a lot less boilerplate code than putting a struct
   around this and passing it around these and passing them around.
   
   If it makes you feel better, imagine this is Java, and this file is
   contains a class defintion and they're just members of the
   class. For a single file, that's pretty much equivalent

*/

int output_level = DEFAULT_OUTPUT_LEVEL;

/* This is the ugly data we get as input.

   I use doubles because FFTW defaults to using doubles, and this
   program runs in realtime on the slowest Linux box I had around. Not
   only do I use ridiculous precision, later you'll find out I
   oversample ridiculously.

   I don't entirely understand the implications of using a lower
   sample rate or lower precision. So, I don't do it.
 */
double *audio_buffer;
size_t audio_buffer_offset;
size_t audio_buffer_length;
size_t audio_buffer_section;
int    audio_eof;
int    framed = 0;

/* The window applied to the input data before we convert it into the
 frequency domain */
double *window;

/* A circular buffer with the difference in power of the two
   frequencies at the last several sample points*/
double *power_diffs;
size_t power_diffs_start = 0;

/* A buffer with a history of total signal strength in for
   the frequencies we care about */
double *power_sq_totals;
size_t power_sq_totals_pos = 0;
double ave_signal_power_sq = 0.0;


/* =======================================================
                         Functions
   ======================================================= */

/* C requires that functions are defined before you reference them, so
   this program ends with the main entry point, and both the playback
   and the record section start with narrow specific functions and
   work toward the most general */



/* These wrapper functions control the behavior of the output.  Normal
   messages are redirected to stderr when you're using the stdout for
   data.

   If you want cosby_debug() to work, you need to redefine
   DEFAULT_OUTPUT_LEVEL at the top of the file */
void cosby_print(char *format, ...){
  va_list ap;
  FILE *out;
  if (output_level & OUTPUT_STDERR)
    out = stderr;
  else
    out = stdout;
  va_start(ap,format);
  vfprintf(out, format, ap);
  va_end(ap);    
}
void cosby_debug(char *format, ...){
  va_list ap;
  if (output_level & OUTPUT_DEBUG) {
    va_start(ap,format);
    vfprintf(stderr, format, ap);
    va_end(ap);    
  }
}
void cosby_print_err(char *format, ...){
  va_list ap;
  va_start(ap,format);
  vfprintf(stderr, format, ap);
  va_end(ap);    
}

/* =======================================================
                         Playback
   ======================================================= */

/* Generates a single wave of the low frequency (zero) and two waves
   of the high frequency (one) using FM Synthesis.

   This is what you came to see, so let's get into it. */
void make_output_audio(double **zero_audio, double **one_audio, size_t low_wavelength) {
  /* FFTW needs to make a "plan" for the best way to solve the FFT for
     particular dimensions before it can solve it. */
  fftw_plan make_waves;

  /* The harmonics we'll use to create our waves. */
  fftw_complex *harmonics; 
  size_t num_harmonics;

  /* Allocate memory for a single wave at the low wavelength.  This
   fits two waves at the higher frequency. */
  (*zero_audio) = (double*)fftw_malloc(sizeof(double)*low_wavelength);
  (*one_audio) = (double*)fftw_malloc(sizeof(double)*low_wavelength);

  /* The number of harmonics necessary to represent a wave is always
     wavelength/2+1. */
  num_harmonics = low_wavelength/2+1;
  harmonics = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*num_harmonics);

  /* Create a plan to turn the harmonics into the zero symbol audio */
  make_waves = fftw_plan_dft_c2r_1d(low_wavelength, harmonics, (*zero_audio),
				    FFTW_ESTIMATE);

  /* Initialize the harmonics */
  for (int c=0;c<num_harmonics;c++) {
    harmonics[c][0] = 0.0;
    harmonics[c][1] = 0.0;
  }

  /* Now for the tricky part. How do will fill in this array
     of harmonics to get what we want?

     We want to create one sine wave at the low wavelength with peaks at -1
     and 1.
  
     The 0th value in our array of harmonics is the part with no
     frequency (the average of the input values), so we don't want
     that. The 1st sample is exactly one wave, which is what we want.
     
     But, what's the right phase and sign?

     The "real" part of the harmonic, the first value, is the cosine
     wave, which begins at its maximum. The second value, the
     "imaginary" part is a sine wave, which begins at zero.  We want a
     pure sine wave, so we'll only set the second value.
     
     The sign of the sine wave determines whether it goes into the negative
     before going positive or vice-versa.

     It doesn't really matter which way it goes. I like waves that go up
     then come down, so I'm setting the value to negative. 

     Bonus: Why is the magnitude 0.5?
  */
  harmonics[1][1] = -0.5;

  /* Run the plan we made earlier on the harmonics we just set up
     to fill in the zero audio */
  fftw_execute(make_waves);

  
  /* Make a new plan for the one audio */
  fftw_destroy_plan(make_waves);
  make_waves = fftw_plan_dft_c2r_1d(low_wavelength, harmonics, (*one_audio),
				    FFTW_ESTIMATE);

  /* Clean up */
  for (int c=0;c<num_harmonics;c++) {
    harmonics[c][0] = 0.0;
    harmonics[c][1] = 0.0;
  }

  /* We want to create a wave with exactly twice the frequency of the
     first wave, which will create exactly two sine waves with peaks
     at -1 and 1.

     That wave is at the second position in our array.  The third value
     in our array corresponds with a wave with three times the frequency,
     the fourth four times the frequency, and so on.

     We set the phase, magnitude, and sign exactly as we did before.

     Now that you understand it, here's something to confuse you:
     In musical terms, this is the first harmonic of the wave we
     created above. It's labeled number two because musicians start
     counting from the base frequency, which mathematicians start
     counting from no frequency.
*/
  harmonics[2][1] = -0.5;

  /* Do the dirty deed */
  fftw_execute(make_waves);

  /* Free up everything we're not using */
  fftw_destroy_plan(make_waves);
  fftw_free(harmonics);
}

/* Free up the memory from the waves created above */
void free_audio_output(double *zero_audio, double *one_audio) {
  fftw_free(zero_audio);
  fftw_free(one_audio);
}

/* You don't need C++ to get polymorphism. These functions give the
   same interface for direct output to the speakers and file
   output. */

/* Output to a file is really simple. */
int output_to_file(void *out_file, double *samples, size_t count) {
  return sf_writef_double((SNDFILE *)out_file,samples,count);
}

/* output to a speaker requires converting samples to 16bit
   because your soundcard probably uses those */
int output_to_speaker(void *device, double *samples, size_t count) {
  short *short_samples;
  int result = 0;
  int err;

  /* From the alloca() man page: The alloca() function is machine and
     compiler dependent. On many systems its implementation is
     buggy. Its use is discouraged. 

     ALSA require alloca, so I'm not worried about it on Linux. */
  short_samples = alloca(sizeof(short)*count);

  for (int c=0;c<count;c++) {
    short_samples[c] = (short)(32767*samples[c]);
  }
  if ((err = snd_pcm_writei(device,(const void *)short_samples, count)) < 0) {
    snd_pcm_prepare(device);
    cosby_debug("Output troubles... %d\n",err);
    result = 1;
  }
  return result;
}

/* Opens a wave file, and puts the handle in out_file */
int init_file_output(void **out_file, char *wave_filename) {
  SF_INFO file_info;
  file_info.frames = 4096;
  file_info.samplerate = DEFAULT_SAMPLE_RATE;
  file_info.channels = 1;
  file_info.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16 | SF_ENDIAN_LITTLE;
  file_info.sections = 0;
  file_info.seekable = 0;
  (*out_file) = (void *)sf_open(wave_filename, SFM_WRITE, &file_info);
  if (out_file == NULL) {
    cosby_print_err( "OUT_FILE ERROR %d\n",sf_error((*out_file)));
    return -1;
  }
  return 0;
}

/* opens your speaker for output, and put the handle in device */
int init_speaker_output(void **device) {
  snd_pcm_hw_params_t *hwparams;
  snd_pcm_format_t format;
  unsigned int rate = DEFAULT_SAMPLE_RATE;
  int err;

  /* ALSA is... complicated.

     Writing code to list out all of the sound devices so the user can
     pick one using a sane name (not something like plughw:0,0) and
     then trying to find a mode with direct hardware support would
     probably be as much work as this entire program.

     I can't find an example of someone doing "the right thing."  They
     all start by assuming you want the default device and 44.1kHz
     16bit stereo interleaved audio. So, in true open source fashion,
     I punted and took the easy way out like everyone else.

     I guess I could provide a way to specify an obscure string for an
     alternate sound device, so there's *some* flexibility. But most
     Linux audio programs don't even bother, so I won't.  And, if you
     know what those strings mean, you're certainly smart enough to
     change the value and recompile.

     Speaking of complicated. What's going on that can't I declare a
     snd_pcm_hw_params_t on the stack? 
   */
  snd_pcm_hw_params_alloca(&hwparams);
  if (is_bigendian())
    format = SND_PCM_FORMAT_S16_BE;
  else
    format = SND_PCM_FORMAT_S16_LE;
  if ((err = snd_pcm_open((snd_pcm_t **)device, ALSA_AUDIO_DEVICE, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
    cosby_print_err( "Uhhh... I don't think this computer has speakers (%d)\n",err);
    return -1;
  }

  if ((err = snd_pcm_hw_params_any((snd_pcm_t *)(*device), hwparams)) < 0) {
    cosby_print_err("Your sounds is really fd up dude. (%d)\n",err);
    return -1;
  }
  if ((err = snd_pcm_hw_params_set_rate_resample((snd_pcm_t *)(*device), hwparams, 0)) < 0) {
    cosby_print_err("Dude! I just said it's fine to resample and your soundscared freaked out on me (%d)\n",err);
    return -1;
  }
  if ((err = snd_pcm_hw_params_set_format((snd_pcm_t *)(*device), hwparams, format)) < 0) {
    cosby_print_err("Hey there. You don't support 16bit sound? Is this 1991? (%d)\n",err);
    return -1;
  }
  if ((err = snd_pcm_hw_params_set_channels((snd_pcm_t *)(*device), hwparams, 1)) < 0) {
    cosby_print_err("Dude! WTF! No mono sound! (%d)\n",err);
    return -1;
  }
  if ((err = snd_pcm_hw_params_set_access((snd_pcm_t *)(*device), hwparams, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
    cosby_print_err("ALSA is seriously braindead. Consider writing a letter to the mailing list complaining. (%d)\n",err);
    return -1;
  }
  if ((err = snd_pcm_hw_params_set_rate_near((snd_pcm_t *)(*device), hwparams, &rate, 0)) < 0) {
    cosby_print_err("I think you have a sound card from 1990 (%d)\n",err);
    return -1;
  }
  if ((err = snd_pcm_hw_params((snd_pcm_t *)(*device), hwparams) < 0)) {
    cosby_print_err("Yur soundscard is br0kn!! (%d)\n",err);
    return -1;
  }

  return 0;
}


/* Get the nth binary digit in a byte. This is what spilts the input
   data into zeros and ones */
inline int get_nth_bit(char byte, size_t n) {
  return (byte & (1 << n));
}

/* Initialize. Play out what we need to. Get out. */
int press_play(char *data_filename, char *wave_filename) {
  double *one_audio;
  double *zero_audio;
  void *out_file;
  FILE *in_file;
  char cur_byte;
  int is_pos;
  int (*output_samples)(void *,double *,size_t);

  make_output_audio(&zero_audio, &one_audio,DEFAULT_WAVELENGTH);

  /* If there's no filename, use the standard input */
  if (data_filename == NULL)
    in_file = stdin;
  else
    in_file = fopen(data_filename,"r");

  if (in_file == NULL) {
    cosby_print_err("Couldn't open %s\n",data_filename);
    return -1;
  }
  if (wave_filename == NULL) {
    output_samples = &output_to_speaker;
    if (init_speaker_output(&out_file) < 0)
      return -1;
  } else {
    output_samples = &output_to_file;
    init_file_output(&out_file, wave_filename);
  } 
  /* Output five seconds of 0 */
  for (int c=0;c<DEFAULT_SAMPLE_RATE*5/DEFAULT_WAVELENGTH;c++) {
    output_samples(out_file,zero_audio,DEFAULT_WAVELENGTH);
  }

  /* Output a byte of all 1s */
  for (int n=7;n>=0;n--) {
    output_samples(out_file,one_audio,DEFAULT_WAVELENGTH/2);
  }
  is_pos = 1;

  /* Loop through the data and output the appropriate
     portion of the appropriate wave.

     If you understand this part, you pretty much understand
     playback. You could just take the sine instead of all that fancy
     FFT bull. FFTW is at its most ineffecient when it's generating a
     single wave. It's expensive to make the plan, so it's usually
     used by running the same plan over and over.
  */
  while (fread(&cur_byte,1,1,in_file)) {
    for (int n=7;n>=0;n--) {
      if (get_nth_bit(cur_byte,n)) {
	if (is_pos) {
	  /* positive one */
	  output_samples(out_file,one_audio,DEFAULT_WAVELENGTH/2);
	} else {
	  /* negative one */
	  output_samples(out_file,one_audio+DEFAULT_WAVELENGTH/4,DEFAULT_WAVELENGTH-DEFAULT_WAVELENGTH/2);
	}
      } else {
	if (is_pos) {
	  /* positive zero */
	  output_samples(out_file,zero_audio,DEFAULT_WAVELENGTH/2);
	} else {
	  /* negative zero */
	  output_samples(out_file,zero_audio+DEFAULT_WAVELENGTH/2,DEFAULT_WAVELENGTH-DEFAULT_WAVELENGTH/2);
	}
	is_pos = !is_pos;
      }
    }
  }

  /* and an extra half a wave for padding */
  if (is_pos) {
    /* positive zero */
    output_samples(out_file,zero_audio,DEFAULT_WAVELENGTH/2);
  } else {
    /* negative zero */
    output_samples(out_file,zero_audio+DEFAULT_WAVELENGTH/2,DEFAULT_WAVELENGTH-DEFAULT_WAVELENGTH/2);
  }

  if (wave_filename == NULL) {
    /* FIXME You forgot to close the soundcard on the way out, you jerk! */
  } else {
    sf_close((SNDFILE *)out_file);
  }

  /* Close the input file, free the audio */
  if (data_filename != NULL)
    fclose(in_file);
  free_audio_output(zero_audio, one_audio);
  return 0;
}

/* =======================================================
                        Record
   ======================================================= */


/* This is tricky.

See, if you have an audio signal segment just starting at some random
place, it will have very sharp edges.  That causes problems. For one,
those edges create the illusion of a higher frequency - they just drop
off suddenly. 

So, we multiply the input by a window function that sort of masks
off the edges. There are several different ones to choose from,
with subtle differences. */
void apply_window_func(double *audio_samples) {
  /* Applies the window function */
  for (int c=0;c<DEFAULT_WAVELENGTH;c++) {
    audio_samples[c] *= window[c];
  }
}


/* =======================================================

Cosby just saves the raw bytestream it generates. It doesn't look
at the contents.

It would be nice to support saving and loading the the WIN994a .TITape
format, so you could work on basic programs using the emulator, then
load them up on a real TI.

I at least figured out the header.  Each .TITape file begins with a 16
byte header

The first 8 bytes of the header are always 5449 2d 54 41 50 45 00
or the 7 ASCII characters "TI-TAPE" followed by a 0. 

For an empty tape, the next 8 bytes are 0s.

The next four bytes appear to be a big endian 32bit unsigned
with the program counter. These can always be 0.

The final four bytes of the header are the length of the file,
also a big endian 32bit unsigned int.

This is different from the header in the actual output.

The "code" in simple BASIC programs saved from an actual TI99/4a and
the emulator is different. It's not clear to me exactly what the
difference is, and I haven't been able to successfully craft a .TITape
file by hand.

   ======================================================= */

/* When cosby sees a "zero" or a "one" in the input,
   it calls this function, which translates it into
   bytes. 

   It looks for a "header" consisting of at least
   eight zeros followed by exactly eight ones.

   Actual tapes contain about five full seconds of zeros
   at the start. */
void process_bit(int bit, FILE *out_file) {
  static char val = 0;
  static int count = 0;
  static int initzeros = 0;
  static int initones = 0;
  if (framed) {
    /* cosby_print_err("%d!!\n",bit); */
    count++;
    val *= 2;
    val += bit;
    if (count == 8) {
      fwrite(&val,1,1,out_file);
      /*  Uncomment this so hitting CTRL-C to stop recording works. */
      /* fflush(out_file); */
      count = 0;
      val = 0;
    }
  } else {
    if (initzeros < 8) {
      if (bit == 0)
	initzeros++;
      else
	initzeros = 0;
    } else {
      if (bit == 1) {
	initones++;
	if (initones == 8) {
	  framed = 1;
	  cosby_print("Got a signal!\n");
	}
      } else if (initones>0) {
	initones = 0;
	initzeros = 1;
      }
    }
  }
}

/* This is where all the magic happens. This is called once
   per sample. Since the FFT is performed over an entire wavelength,
   it's probably overkill. */
int process_harmonics(fftw_complex *harmonics, size_t num_harmonics, FILE *out_file) {
  static int current_symbol = 1; /* The bit symbol we're currently looking at */
  static int sample_count = 0; /* Samples we've seen in this symbol */
  double ave_power_diff = 0.0;
  double ave_power_total_sq = 0.0;

  power_sq_totals[power_sq_totals_pos++] = (harmonics[1][0]*harmonics[1][0]+
					    harmonics[1][1]*harmonics[1][1]+
			    (harmonics[2][0]*harmonics[2][0]+
			     harmonics[2][1]*harmonics[2][1]));
  if (power_sq_totals_pos >= POWER_SQ_TOTALS_SIZE*DEFAULT_SYMBOL_LENGTH) {
    for (int c=0;c<POWER_SQ_TOTALS_SIZE*DEFAULT_SYMBOL_LENGTH;c++)
      ave_power_total_sq += power_sq_totals[c]/(POWER_SQ_TOTALS_SIZE*DEFAULT_SYMBOL_LENGTH);
    if (framed) {
      if (ave_signal_power_sq == 0.0) {
	ave_signal_power_sq = ave_power_total_sq;
	/* cosby_print_err("Power: %f\n",ave_power_total_sq); */
	/* SIGNAL_POWER_RANGE needs to be sqaured to compare
	   sqaured powers */
      } else if (ave_power_total_sq*SIGNAL_POWER_RANGE*SIGNAL_POWER_RANGE < ave_signal_power_sq) {
	return 1;
      }
    }
    power_sq_totals_pos = 0;
  }
  power_diffs[power_diffs_start++] = sqrt(harmonics[1][0]*harmonics[1][0]+harmonics[1][1]*harmonics[1][1])-
    sqrt(harmonics[2][0]*harmonics[2][0]+harmonics[2][1]*harmonics[2][1]);
  if (power_diffs_start>=DEFAULT_SYMBOL_LENGTH/2)
    power_diffs_start = 0;

  sample_count++;

  for (int c=0;c<DEFAULT_SYMBOL_LENGTH/2;c++) {
    ave_power_diff+=power_diffs[c];
  }
  ave_power_diff/=DEFAULT_SYMBOL_LENGTH/2;

  if (current_symbol == 1 && ave_power_diff > 0.0) {
    current_symbol = 0;
    sample_count = 0;
    process_bit(0, out_file);
  } else if (current_symbol == 0 && ave_power_diff < 0.0) {
    current_symbol = 1;
    sample_count = 0;
    process_bit(1, out_file);
  }  else if (sample_count > (int)(1.5*DEFAULT_SYMBOL_LENGTH)) {
    process_bit(current_symbol, out_file);
    sample_count -= DEFAULT_SYMBOL_LENGTH;
  }
  return 0;
}

/* Creates a half-wave window function, (which is called a Hamming
window function) one wave long. It may be possible to use a correctly
tuned Doplh-Chebyshev or some other foreign name to get better
performance. */
int init_window() {
  window = (double*)fftw_malloc(sizeof(double)*DEFAULT_WAVELENGTH);
  if (window == NULL)
    return 1;
  for (int c=0;c<DEFAULT_WAVELENGTH;c++) {
    window[c] = cos(c/(DEFAULT_WAVELENGTH-1.0)*PI-PI/2);
  }
  return 0;
}

/* The arrays of power differences and totals */
int init_history() {
  power_diffs = fftw_malloc(sizeof(double)*(DEFAULT_SYMBOL_LENGTH/2));
  if (power_diffs == NULL)
    return 1;
  for (int c=0;c<DEFAULT_SYMBOL_LENGTH/2;c++) {
    power_diffs[c] = 0.0;
  }
  power_sq_totals = fftw_malloc(sizeof(double)*(POWER_SQ_TOTALS_SIZE*DEFAULT_SYMBOL_LENGTH));
  if (power_sq_totals == NULL)
    return 1;
  for (int c=0;c<POWER_SQ_TOTALS_SIZE*DEFAULT_SYMBOL_LENGTH;c++) {
    power_sq_totals[c] = 0.0;
  }
  return 0;

}

/* This is a circular buffer. That means, if you reach the end,
you just start again at the beginning. */
int init_audio_buffer(int (*read_samples)(void *device, double *buffer, size_t count), void *in_file) {
  audio_buffer_offset = 0;
  audio_buffer_length = 0;
  audio_buffer_section = 0;
  audio_eof = 0;
  audio_buffer = (double*)fftw_malloc(sizeof(double)*AUDIO_BUFFER_SIZE);
  if (audio_buffer != NULL) {
    audio_buffer_length = (*read_samples)(in_file, audio_buffer, AUDIO_BUFFER_SIZE);
    if (audio_buffer_length != AUDIO_BUFFER_SIZE) {
      audio_eof = 1;
    }
    return 0;
  } else {
    return 1;
  }
}

/* free up those resources */
void free_audio_buffer(SNDFILE *in_file) {
  sf_close(in_file);
  fftw_free(audio_buffer);
}
void free_history() {
  fftw_free(power_diffs);
  fftw_free(power_sq_totals);
}
void free_window() {
  fftw_free(window);
}

/* 
    This function puts length samples of audio starting at offset,
    into out, and returns the number of samples read.

    This is why C is fun. This function returns samples for either an
    audio device or an audio file.  The calling function passes in an
    appropriate read_samples.
   
    It's pretty complicated and inefficient because it copies over
    bytes from the circular buffer, rather than using a more complex
    data structure that wouldn't require copying.

    Since this doesn't use much CPU power relative to the FFT, I
    figure it's better to be inefficient.
*/

int audio_at_offset(
            int (*read_samples)(void *device, double *buffer, size_t count),
	    void *in_file, double *out, size_t offset, size_t length) {
  int count;
  if (length > AUDIO_BUFFER_SIZE)
    return -1; /* Buffer too small */

  if (offset < audio_buffer_offset)
    return -1; /* No going backwards in the file */


  /* Loop as long as we still need to read from the file */
  while (!audio_eof &&
      (audio_buffer_offset+audio_buffer_length) < (offset+length)) {

    /* We're starting in the second section */
    if (audio_buffer_section == 1) {
      count = (*read_samples)(in_file, audio_buffer+AUDIO_BUFFER_SIZE/2,
			     AUDIO_BUFFER_SIZE/2);
      audio_buffer_section = 0;
      audio_buffer_offset+=AUDIO_BUFFER_SIZE/2;
      if (count < AUDIO_BUFFER_SIZE/2) {
	audio_eof = 1;
	audio_buffer_length -= (AUDIO_BUFFER_SIZE/2-count);
      }
    } else {
      count = (*read_samples)(in_file, audio_buffer,
			     AUDIO_BUFFER_SIZE/2);
      audio_buffer_offset+=AUDIO_BUFFER_SIZE/2;
      audio_buffer_section = 1;
      if (count < AUDIO_BUFFER_SIZE/2) {
	audio_eof = 1;
	audio_buffer_length -= (AUDIO_BUFFER_SIZE/2-count);
      }
    }
  }
  /* If we're on the first section, just copy the bytes over */
  if (audio_buffer_section == 0) {

    /* There aren't enough bytes left, fill the remainder with zeros */
    if (audio_buffer_length+audio_buffer_offset-offset<length) {
      count = audio_buffer_length+audio_buffer_offset-offset;
      memcpy(out, audio_buffer+offset-audio_buffer_offset, count*sizeof(double));
      memset(out+count,0,(length-count)*sizeof(double));
      return count;
    } else { /* There are enough bytes, just copy it and be done */
      memcpy(out, audio_buffer+offset-audio_buffer_offset, length*sizeof(double));
      return length;
    }
  } else { /* We're in the second section, it's more complicated */

     /* Find the total number of bytes to copy */
    if (audio_buffer_length+audio_buffer_offset-offset<length)
      count = audio_buffer_length+audio_buffer_offset-offset;
    else
      count = length;

    /* We're starting in the second section */
    if (offset-audio_buffer_offset<AUDIO_BUFFER_SIZE/2) {
      /* and it wraps around. */
      if (offset-audio_buffer_offset+count>AUDIO_BUFFER_SIZE/2) {
  	memcpy(out, audio_buffer+AUDIO_BUFFER_SIZE/2+offset-audio_buffer_offset,
  	       (AUDIO_BUFFER_SIZE/2-(offset-audio_buffer_offset))*sizeof(double));
  	memcpy(out+AUDIO_BUFFER_SIZE/2-(offset-audio_buffer_offset),
	       audio_buffer,(count-(AUDIO_BUFFER_SIZE/2-
  				    (offset-audio_buffer_offset)))*sizeof(double));
      } else { /* it doesn't wrap around */
  	memcpy(out, audio_buffer+AUDIO_BUFFER_SIZE/2+offset-audio_buffer_offset,
  	       count*sizeof(double));
      }
    } else { /* We're starting in the first section, just copy and be done */
      memcpy(out, audio_buffer+offset-audio_buffer_offset-AUDIO_BUFFER_SIZE/2, count*sizeof(double));
    }
    memset(out+count,0,(length-count)*sizeof(double));
    return count;
  }
}

int read_from_file(void *in_file, double *buffer, size_t count) {
  return sf_read_double((SNDFILE *)in_file, buffer, count);
}

int read_from_mic(void *device, double *samples, size_t count) {
  static size_t total_read=0;
  short short_samples[AUDIO_BUFFER_SIZE*2];
  int err;
  /*  char c; */

  /* FIXME: add manual keyboard shutdown */
  /* if (read(STDIN_FILENO,&c,1) != 0) */
  /*   return 1; */

  if (count>AUDIO_BUFFER_SIZE) {
    cosby_print_err("Reading too much\n");
    return -1;
  }
  if ((err = snd_pcm_readi(device,(void *)short_samples, count)) < 0) {
    snd_pcm_prepare(device);
    cosby_print_err("Input troubles... %d\n",err);
    return 0;
  }
  for (int c=0;c<count;c++) {
    /* Only look at the left channel, even though we get both as input */
    samples[c] = (double)(short_samples[c*2]);
  }
  total_read += count;
  if (total_read > DEFAULT_SAMPLE_RATE*MAX_WAIT && !framed) {
    cosby_print("No signal found. Giving up.\n");
    return -2;
  }
  return count;
}

int init_file_input(void **in_file, char *wave_filename) {
  SF_INFO file_info;
  memset((void *)&file_info,0,sizeof(SF_INFO));
  (*in_file) = sf_open(wave_filename, SFM_READ, &file_info);
  if (file_info.samplerate != DEFAULT_SAMPLE_RATE){
    cosby_print_err("Sorry, this program is lame and only supports %d samples per second\n",DEFAULT_SAMPLE_RATE);
    return -1;
  } else if (file_info.channels != 1) {
    cosby_print_err("Sorry, this program is lame and only supports mono .WAV files\n");
    return -1;
  }

  return 0;
}
int init_mic_input(void **device) {
  snd_pcm_hw_params_t *hwparams;
  snd_pcm_format_t format;
  unsigned int rate = DEFAULT_SAMPLE_RATE;
  int err;
  
  snd_pcm_hw_params_alloca(&hwparams);
  if (is_bigendian())
    format = SND_PCM_FORMAT_S16_BE;
  else
    format = SND_PCM_FORMAT_S16_LE;
  if ((err = snd_pcm_open((snd_pcm_t **)device, ALSA_AUDIO_DEVICE, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
    cosby_print_err("Uhhh... I don't think this computer has speakers (%d)\n",err);
    return -1;
  }

  if ((err = snd_pcm_hw_params_any((snd_pcm_t *)(*device), hwparams)) < 0) {
    cosby_print_err("Your sounds is really fd up dude. (%d)\n",err);
    return -1;
  }
  if ((err = snd_pcm_hw_params_set_rate_resample((snd_pcm_t *)(*device), hwparams, 0)) < 0) {
    cosby_print_err("Dude! I just said it's fine to resample and your soundscared freaked out on me (%d)\n",err);
    return -1;
  }
  if ((err = snd_pcm_hw_params_set_format((snd_pcm_t *)(*device), hwparams, format)) < 0) {
    cosby_print_err("Hey there. You don't support 16bit sound? Is this 1991? (%d)\n",err);
    return -1;
  }
  if ((err = snd_pcm_hw_params_set_channels((snd_pcm_t *)(*device), hwparams, 2)) < 0) {
    cosby_print_err("Dude! WTF! No stereo sound! (%d)\n",err);
    return -1;
  }
  if ((err = snd_pcm_hw_params_set_access((snd_pcm_t *)(*device), hwparams, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
    cosby_print_err("ALSA is seriously braindead. Consider writing a letter to the mailing list complaining. (%d)\n",err);
    return -1;
  }
  if ((err = snd_pcm_hw_params_set_rate_near((snd_pcm_t *)(*device), hwparams, &rate, 0)) < 0) {
    cosby_print_err("I think you have a sound card from 1990 (%d)\n",err);
    return -1;
  }
  if ((err = snd_pcm_hw_params((snd_pcm_t *)(*device), hwparams) < 0)) {
    cosby_print_err("Yur soundscard is br0kn!! (%d)\n",err);
    return -1;
  }
  return 0;
}

int press_record(char *data_filename, char *wave_filename) {
  /* The overall goal here is to seamlessly decode as many different audio
     inputs as possible.

     There's a lot of subtlety to decoding analog input with an FFT,
     especially for a protocol like this one with a symbol that's
     half a wavelength.

     Instead of making proper hardware to amplify and filter the
     output of the TI99/4a, I made this software capable of decoding
     the weak, noisy crap obtained from connecting the microphone
     output to a line-in without any filter or amplifier.

     I perform an FFT over one wavelength once per sample. Computing
     the FFT for every sample is probably overkill, but it's
     significantly faster than realtime even on an old PowerBook G4,
     and it doesn't hurt to average out noise better.

     Using a buffer size of one wavelength means that the first and
     second harmonics are centered at exactly the frequencies we're
     looking at. I believe it's necessary to use a window function, so
     that alternating 0s and ones can be differentiated. I'm using the
     Hamming window (half a sine wave), but there may be better ones.
     
     There can be a lot of noise in the signal, so I keep a
     half-symbol size buffer of the difference between the signal
     strengths of the two frequencies and use the average.  (It it was
     a whole buffer, it would blur out the boundaries between
     symbols.)  If the difference between the low and high and low
     frequncies drops below or rises above the midpoint, it triggers a
     new signal.

*/


  void *in_file;
  FILE *out_file;
  size_t num_harmonics;
  sf_count_t samples_read;
  fftw_complex *harmonics;
  fftw_plan get_frequencies;
  size_t offset = 0;
  double *audio_samples;
  int (*read_samples)(void *device, double *buffer, size_t count);

  /* Initialize the FFT

     If the FFT is over the wavelength of the low frequency (twice the symbol size)

     sample 0 - DC
     sample 1 - wavelen of 2 symbols - "Zero"
     sample 2 - wavelen of 1 symbol - "One"

     This doesn't have a very narrow filter, so it might be susceptable to interference.     
   */
  num_harmonics = DEFAULT_WAVELENGTH/2+1;
  harmonics = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*num_harmonics);

  audio_samples = (double*) fftw_malloc(sizeof(double)*DEFAULT_WAVELENGTH);
  if (wave_filename == NULL) {
    read_samples = &read_from_mic;
    init_mic_input(&in_file);

  } else {
    read_samples = &read_from_file;
    init_file_input(&in_file,wave_filename);
  }
  init_audio_buffer(read_samples, in_file);
  init_history();
  init_window();
  if (data_filename == NULL)
    out_file = stdout;
  else
    out_file = fopen(data_filename,"wb");

  get_frequencies = fftw_plan_dft_r2c_1d(DEFAULT_WAVELENGTH, audio_samples,
					 harmonics,
					 FFTW_ESTIMATE | FFTW_DESTROY_INPUT);


  while ((samples_read = audio_at_offset(read_samples, in_file, audio_samples, offset++, DEFAULT_WAVELENGTH))>0) {
    apply_window_func(audio_samples);
    fftw_execute(get_frequencies);
    if (process_harmonics(harmonics, num_harmonics, out_file))
      break;
  }
  cosby_print("Done!\n");

  if (data_filename != NULL)    
    fclose(out_file);
 
  fftw_destroy_plan(get_frequencies);

  fftw_free(harmonics);
  fftw_free(audio_samples);
  if (wave_filename == NULL) {
  } else {
    free_audio_buffer(in_file);
  }
  free_history();
  free_window();

  return 1;
}

/* =======================================================
                       Entry point
   ======================================================= */

/* Parse the arguments and invoke either play or record */
int main(int argc, char *argv[]) {
  int result;
  if ((argc>=3 && argc <= 5) &&
      0==strcmp(argv[1],"press") &&
      0==strcmp(argv[2],"record")) {
    if (argc == 3 || (argv[3][0]=='-' && argv[3][1] == 0)) {

      /* Redirect all messages to stderr */
      output_level |= OUTPUT_STDERR;
      if (argc == 5) {
	cosby_print("Recording %s to stdout\n",argv[4]);
	result = press_record(NULL,argv[4]);
      } else {
	cosby_print("Recording to stdout\n");
	result = press_record(NULL,NULL);
      }
    } else if (argc == 4) {
	cosby_print("Recording to %s\n",argv[3]);
	result = press_record(argv[3],NULL);
    } else {
	cosby_print("Recording %s to %s\n",argv[4],argv[3]);
	result = press_record(argv[3],argv[4]);
    }
  } else if ((argc>=3 && argc <= 5) &&
	     0==strcmp(argv[1],"press") &&
	     0==strcmp(argv[2],"play")) {
    
    if (argc == 3 || (argv[3][0]=='-' && argv[3][1] == 0)) {
      if (argc == 5) {
	cosby_print("Playing stdin to %s\n",argv[4]);
	result = press_play(NULL,argv[4]);
      } else {
	cosby_print("Playing stdin\n");
	result = press_play(NULL,NULL);
      }
    } else if (argc == 4) {
      cosby_print("Playing out %s\n",argv[3]);
      result = press_play(argv[3],NULL);
    } else {
      cosby_print("Playing out %s to %s\n",argv[3],argv[4]);
      result = press_play(argv[3],argv[4]);
    }

  } else {    
    cosby_print("Cosby is TI99/4a data cassette interface software modem \n\n");
    cosby_print("Usage: %s press record <output.dat> [<input.wav>]\n",argv[0]);
    cosby_print("       %s press play <input.dat> [<output.wav>]\n",argv[0]);
    cosby_print("\n  Hint: '-' as <output.dat> or <input.dat> for stdin and stdout\n");
    result = 1;
  }
  return result;
}
