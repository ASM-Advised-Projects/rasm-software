/**
 * Defines the Display4D class for controlling a 4D Systems Diablo gen4 no-
 * touch display module.
 */

#ifndef DISPLAY_4D_H
#define DISPLAY_4D_H

#include "diablo_commands.h"

#include "periphery_access.hpp"
#include "periphery_config.h"
#include "../configuration.hpp"

#include "periphery/serial.h"

#include <Poco/Thread.h>

#include <string>
#include <array>
#include <queue>

using std::string;

/**
 * This class provides a simple control portal for a 4D Systems gen4 display
 * module mounted on the RASM. There are three screens that can be used: blank,
 * splash, and main. The splash screen will display a large 'RASM' along with
 * optional loading messages. The layout of the main screen is shown below:
 *  -----------------------------------------
 * |          Mode: xxxxxxxxxxxxxx           |
 * | Battery Status       | Message          |
 * |   _____________      | xxxxxxxxxxxx     |
 * |  |    xxx%     |     |                  |
 * |   -------------      |                  |
 * | - - - - - - - - - - -|                  |
 * | Wifi Access Point    |- - - - - - - - - |
 * | Name: xxxxxxx        | xxxxxxxxxxxxxxx  |
 * | Passphrase: xxxxxxx  |           -----> |
 *  -----------------------------------------
 * The x's can be whatever characters. They are set by the set methods.
 * The right-pointing arrow in the lower-right corner is meant to point at a
 * physical button that should be located close the diplay edge where it's
 * pointing.
 *
 * The text is displayed in landscape mode with a black background and white
 * text. This goes for both the splash screen and main screen. The blank screen
 * will have the black background with no content overlaid on it.
 *
 * All methods within this class are non-blocking. The commands that need to be
 * sent to the display and subsequently acknowledged are processed in a
 * background thread.
 *
 * Instances of this class are thread safe.
 */
class Display4D : Poco::Runnable
{
private:
  /**
   * This enumeration lists all of the available baud rates. Their names
   * are just the rates (in Hz) that they represent with prefixed underscores.
   * This list is relatively small because these were the only values that both
   * the display and the usual UART drivers of the linux kernel support.
   *
   * Note that this enum isn't actually used because it's just meant to formally
   * indicate what baud rates are allowed.
   */
  enum _BaudRate { _9600, _19200, _38400, _57600, _115200, _500000 };

  /**
   * A list of messages that can be randomly displayed on the splash screen.
   * Many of these were copied and adapted from this repository:
   * https://github.com/1egoman/funnies/blob/master/src/funnies.js
   */
  const std::array<std::string, 19> splash_messages = {
      string("Configuring the configurator")
    , "Interlacing time and space"
    , "Dividing by zer"
    , "Hacking the mainframe"
    , "Entangling superstrings"
    , "Sorting the potatoes from the non-potatoes"
    , "Capturing the rouge bits"
    , "Reading terms and conditions for you"
    , "Encrypting the self-destruct codes"
    , "Calibrating to the gravitational constant in your locale"
    , "Obfuscating quantum entanglement"
    , "Fluxing the flux capacitor"
    , "*insert quarter*"
    , "Computing the chance of success"
    , "Discovering new ways of making you wait..."
    , "Your patience is being purposefully tested"
    , "We're not liable for any broken hardware as a result of waiting"
    , "… at least you're not on hold"
    , "Still faster than a Windows update, am I right?"
  };

  // for handling splash messages
  std::array<int, 19> splash_msg_indices;
  int splash_msg_index;
  int longest_splash_msg;

  // for processing the command queue
  std::queue<command_t> cmd_queue;
  Poco::Thread cmd_process_thread;
  Poco::Event cmd_event;
  bool exit_thread;

  // for communicating with the display
  command_t command;
  serial_t uart_port;
  bool received;
  int recv_timeout;

  // 0 - blank screen; 1 - splash screen; 2 - main screen
  int screen_state;

  // pixel dimensions
  int screen_width = 480;
  int screen_height = 320;
  int text_width = 12;
  int text_height = 16;

  /**
   * Runs an event-driven processing loop that sends queued commands, one at a
   * time, to the display whenever the display is ready to receive a command.
   * If the queue is empty, then this method will stop execution until cmd_event
   * is signaled.
   */
  void run()
  {
    received = true;
    unsigned char recvbuff[3];

    while (!exit_thread)
    {
      if (cmd_queue.size() > 0)
      {
        if (received)  // if display is ready for a new command
        {
          // send next command (with the ending null character)
          command_t &cmd = cmd_queue.front();
          serial_write(&uart_port, &(cmd.buffer[0]), cmd.txlength);
          serial_flush(&uart_port);
          received = false;
        }
        else
        {
          // wait for and read in the response
          command_t &cmd = cmd_queue.front();
          serial_read(&uart_port, recvbuff, cmd.rxlength, recv_timeout);
          received = true;
          cmd_queue.pop();
        }
      }
      else
      {
        cmd_event.wait();
      }
    }
  }

  /**
   * Queues the contents of the given command for the queue processing thread
   * to send to the display.
   */
  void queue_cmd(command_t &cmd)
  {
    cmd_queue.push(cmd);
    cmd_event.set();
  }

/**
   * Constructs a new Display4D instance to control a display at the configured
   * baud rate and uart device. This constructor will cause the display screen
   * to be blank with a black background. Note that a background thread is
   * started which is used to send queued commands to the display whenever it
   * isn't busy.
   *
   * Throws Poco::RuntimeException if the uart device can't be opened.
   */
  Display4D()
  : cmd_event(true)
  , exit_thread(false)
  , recv_timeout(100)
  {
    Poco::AutoPtr<MapConfiguration> configs = ConfigurationManager::get_instance().
        get_config_group(ConfigurationManager::Group::PERIPHERY);

    // load the uart device index and baud rate from configurations
    int uartbus = configs->getInt("display.uartbus");
    int baudrate = configs->getInt("display.baudrate");

    // find length of the longest splash message and initialize index array
    longest_splash_msg = 0;
    for (int i = 0; i < splash_messages.size(); i++)
    {
      splash_msg_indices[i] = i;
      if (splash_messages[i].length() > longest_splash_msg)
        longest_splash_msg = splash_messages[i].length();
    }

    // randomize the splash message index array
    std::random_shuffle(splash_msg_indices.begin(), splash_msg_indices.end());
    splash_msg_index = 0;

    // start the command queue processing thread (this)
    cmd_process_thread.setName("display4d");
    cmd_process_thread.setPriority(Poco::Thread::PRIO_NORMAL);
    cmd_process_thread.start(*this);

    // initialize the uart port connected to the display (defaut baud = 9600)
    UartConf uartconf;
    bool success = PeripheryAccess::get_instance().init_uart_device(
        uartbus, &uart_port, uartconf);
    if (!success)
    {
      throw Poco::RuntimeException("In Display4D::Display4D()\n"
      "Initialization failed for UART device number " + std::to_string(uartbus) + ".");
    }
    /*struct termios new_port_settings;
    int k, ch, tSave, baudr;
    cPort = open(sDeviceName, O_RDWR | O_NOCTTY | O_NDELAY);
    tcgetattr(cPort, &new_port_settings);
    cfmakeraw(&new_port_settings);  // Set the line to RAW
    memset(&new_port_settings, 0, sizeof(new_port_settings));  // clear the new struct
    new_port_settings.c_cflag = baudr | CS8 | CLOCAL | CREAD;
    new_port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    new_port_settings.c_iflag = IGNPAR;
    new_port_settings.c_oflag = 0;
    new_port_settings.c_lflag = 0;
    new_port_settings.c_cc[VMIN] = 0;      // block untill n bytes are received
    new_port_settings.c_cc[VTIME] = 1;     // block untill a timer expires (n * 100 mSec.)
    tcsetattr(cPort, TCSANOW, &new_port_settings);
    fcntl(cPort, F_SETFL, FNDELAY);*/  // Set non-blocking

    // change the default baud rate of both the display and the computer's uart
    // device to the given rate
    int rateindex;
    switch (baudrate)
    {
      case 9600: rateindex = 6; break;
      case 19200: rateindex = 8; break;
      case 38400: rateindex = 10; break;
      case 57600: rateindex = 12; break;
      case 115200: rateindex = 13; break;
      case 500000: rateindex = 18; break;
      default: rateindex = 13; baudrate = 115200; break;
    }

    /* Can't use the queue system because the uart device's baud rate must be
       changed between the time of the command being sent and the response
       being received.*/
    command_t cmd;
    setbaudWait(&cmd, rateindex);  // set baud rate of display
    serial_write(&uart_port, &(cmd.buffer[0]), cmd.txlength);
    serial_flush(&uart_port);
    serial_set_baudrate(&uart_port, baudrate);  // set baud rate of computer
    unsigned char recvbuff[3];
    serial_read(&uart_port, recvbuff, cmd.rxlength, recv_timeout);

    // basic screen settings
    gfx_ScreenMode(&cmd, LANDSCAPE);  // set display orientation to landscape
    queue_cmd(cmd);
    gfx_BGcolour(&cmd, BLACK);  // set background color to black
    queue_cmd(cmd);
    txt_BGcolour(&cmd, BLACK);  // set text background color to black
    queue_cmd(cmd);
    txt_FGcolour(&cmd, WHITE);  // set text color to white
    queue_cmd(cmd);
    txt_FontID(&cmd, 4);  // set the font to System-12x16
    queue_cmd(cmd);
    txt_Width(&cmd, 2);  // set default text width multiplier
    queue_cmd(cmd);
    txt_Height(&cmd, 2);  // set default text height multiplier
    queue_cmd(cmd);

    showBlankScreen();  // clear the screen
  }

public:
  Display4D(const Display4D &) = delete;
  void operator=(const Display4D &) = delete;

  /**
   * Returns a reference to this singleton's instance.
   */
  static Display4D & get_instance()
  {
    static Display4D display;
    return display;
  }

  /**
   * Destructs this instance.
   */
  ~Display4D()
  {
    serial_close(&uart_port);

    cmd_event.set();
    cmd_process_thread.join();

    cmd_process_thread.~Thread();
    cmd_event.~Event();
    cmd_queue.~queue();

    splash_msg_indices.~array();
  }

  /**
   * Clears the display with a black background color.
   */
  void showBlankScreen()
  {
    command_t cmd;
    gfx_Cls(&cmd);  // clear the screen
    queue_cmd(cmd);
    screen_state = 0;
  }

  /**
   * Displays the splash screen which has a black background with large 'RASM'
   * text slightly up from the center of the screen. The screen is cleared in
   * order to do this. If the splash screen is already displaying then this
   * method does nothing.
   */
  void showSplashScreen()
  {
    if (screen_state == 1)
      return;

    if (screen_state != 0)
      showBlankScreen();

    // display the 'RASM' text
    command_t cmd;
    txt_Bold(&cmd, 1);  // make text bold
    queue_cmd(cmd);
    gfx_MoveTo(&cmd, 100, 50);  // move origin
    queue_cmd(cmd);
    txt_Width(&cmd, 10);  // set text width multiplier
    queue_cmd(cmd);
    txt_Height(&cmd, 10);  // set text height multiplier
    queue_cmd(cmd);
    putstr(&cmd, "RASM");  // print text
    queue_cmd(cmd);

    screen_state = 1;

    // restore the default text sizes
    txt_Width(&cmd, 2);  // set text width multiplier
    queue_cmd(cmd);
    txt_Height(&cmd, 2);  // set text height multiplier
    queue_cmd(cmd);
  }

  /**
   * Randomly selects a splash screen message and displays it under the 'RASM'
   * text. No message will be displayed more than once until all messages have
   * been displayed. If the splash screen is not currently showing then this
   * method will do nothing.
   */
  void randomSplashMessage()
  {
    if (screen_state != 1)
      return;

    // get the current splash message
    string msg = splash_messages[splash_msg_indices[splash_msg_index]];

    // increment the index for the splash message indices array
    splash_msg_index = (splash_msg_index + 1) % splash_msg_indices.size();

    // pad the message with spaces on both sides to make it the size of the
    // largest message while still being centered; this ensures that any
    // previous message will be entirely erased
    int padding = longest_splash_msg - msg.length();
    int right_padding = (padding % 2 == 1) ? padding/2+1 : padding/2;
    msg.insert(0, padding/2, ' ');
    msg.insert(msg.length(), right_padding, ' ');

    // display the splash message
    command_t cmd;
    txt_Bold(&cmd, 0);  // make text regular
    queue_cmd(cmd);
    gfx_MoveTo(&cmd, 50, 250);  // move origin
    queue_cmd(cmd);
    putstr(&cmd, msg.c_str());  // print text
    queue_cmd(cmd);
  }

  /**
   * Displays the main screen with no mode, 0 battery percent, no wifi info,
   * no message, and no button info. If the main screen is already showing then
   * this method does nothing.
   */
  void showMainScreen()
  {
    if (screen_state != 2)
      return;
    screen_state = 2;

    command_t cmd;
    txt_Bold(&cmd, 0);  // make text bold
    queue_cmd(cmd);

    setMode("");

    // battery status header
    gfx_MoveTo(&cmd, 30, 60);  // move origin
    queue_cmd(cmd);
    putstr(&cmd, "Battery Status");  // print text
    queue_cmd(cmd);

    // battery symbol
    gfx_Rectangle(&cmd, 30, 100, 200, 160, WHITE);
    queue_cmd(cmd);
    gfx_Rectangle(&cmd, 200, 120, 210, 140, WHITE);
    queue_cmd(cmd);

    // wifi access point header
    gfx_MoveTo(&cmd, 30, 200);  // move origin
    queue_cmd(cmd);
    putstr(&cmd, "WiFi Access Point");  // print text
    queue_cmd(cmd);

    // message header
    gfx_MoveTo(&cmd, 230, 60);  // move origin
    queue_cmd(cmd);
    putstr(&cmd, "Message");  // print text
    queue_cmd(cmd);

    // button arrow
    gfx_MoveTo(&cmd, 400, 300);  // move origin
    queue_cmd(cmd);
    putstr(&cmd, "----->");  // print text
    queue_cmd(cmd);
  }

  /**
   * Displays the given mode after the 'Mode' label.
   */
  void setMode(const char *mode)
  {
    if (screen_state != 2)
      return;

    command_t cmd;
    gfx_MoveTo(&cmd, 30, 20);  // move origin
    queue_cmd(cmd);
    putstr(&cmd, ("Mode: " + string(mode)).c_str());  // print text
    queue_cmd(cmd);
  }

  /**
   * Displays the given message under the 'Message' header.
   */
  void setMessage(const char *msg)
  {
    if (screen_state != 2)
      return;

  }

  /**
   * Displays a description for the button next to the arrow pointing to it.
   */
  void setLowButtonAction(const char *action)
  {
    if (screen_state != 2)
      return;

  }

  /**
   * Displays the battery percentage and fills in the battery rectangle by an
   * amount that corresponds to that percentage.
   */
  void setBatteryPercent(unsigned int percent)
  {
    if (screen_state != 2)
      return;

  }

  /**
   * Displays the given name and passphrase info under the 'WiFi Access Point'
   * header.
   */
  void setWifiInfo(const char *name, const char *passphrase)
  {
    if (screen_state != 2)
      return;

  }
};

#endif
