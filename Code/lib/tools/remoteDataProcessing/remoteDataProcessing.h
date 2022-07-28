/**
 * @file remoteDataProcessing.h
 * @ingroup tools
 *
 * Interface with a remote data processor (such as MATLAB, Octave, Scilab)
 * through HWPCS. Arbitrary data and commands for processing can be transferred
 * and executed by the remote data processor. This can for example be useful for
 * calibration or collection and processing of experimental data.
 */

#ifndef REMOTEDATAPROCESSING_H_
#define REMOTEDATAPROCESSING_H_

#include <stdbool.h>


/**
 * Send a command on communication channel #CH_OUT_RDP to a remote data
 * processor (for example MATLAB, Octave, Scilab) through HWPCS. The commands
 * are added to a list (a script to be executed by a configured data processor)
 * within HWPCS as long as the parameter execute is false. In order to add
 * the last command and execute the whole script, set the parameter execute
 * to true.
 *
 * @param   execute   false to just add the command to the list,
 *                    true to add the command and execute the whole script
 * @param   command   a string containing the command, for format please see
 *                    <a href="https://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html" target="_blank">vfprintf</a>
 *                    from AVR Libc.
 *                    The resulting string will be truncated after 512 characters
 *                    excluding null termination.
 *
 */
void remoteDataProcessing_command(const bool execute, const char* command, ...);


/**
 * Clear the list of commands (the script) contained in HWPCS. This function
 * should be used in order to start a new script and before commands are added
 * with remoteDataProcessing_command().
 */
void remoteDataProcessing_clear(void);

#endif /* REMOTEDATAPROCESSING_H_ */
