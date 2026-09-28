#!/usr/bin/env python3
# Task 4 owner: Tan Chew Miang Edwin (A0201867A), Group 13.
"""Speech-to-text front end for Task 4, feeding the existing Task 3 mission.

Captures one spoken English colour-search command from the microphone,
transcribes it, prints the required [STT] lines, validates it with the same
parser Task 3 uses for typed commands (task3_core.parse_command), and — only
on a successful parse — publishes the recognised text on /speech_command
(std_msgs/String, matching the 'speech_topic' parameter already wired into
task3_mission.py's Task3MissionNode.on_command(msg, speech=True)). The
mission node then starts the identical autonomous search used for Task 3;
this node does not talk to Nav2, the camera or the vehicle itself.

STT engine: SpeechRecognition (https://pypi.org/project/SpeechRecognition/)
with Google's free Web Speech API (recognize_google, needs internet) by
default, or the offline CMU Sphinx engine (recognize_sphinx, needs
pocketsphinx) selected with the 'engine' parameter. Cite both in the README.

Run directly, the same way as scripts/task3_command.py (not through
`ros2 launch`, so this terminal keeps normal stdin for push-to-talk):

    python3 "$(ros2 pkg prefix ee5112_vehicle)/share/ee5112_vehicle/scripts/speech_command.py" \
        --ros-args --params-file ros2_ws/src/ee5112_vehicle/config/speech_command.yaml

Developed with AI coding assistance.
"""
import sys

from task3_core import parse_command


def classify_transcript(text):
    """Pure decision, no ROS/audio: recognised text -> ('ok', colours) or ('error', reason).

    Reuses task3_core.parse_command with allow_bare_list=True so a spoken
    "red and blue" is accepted exactly like a typed "find red and blue";
    every other acceptance/rejection rule (English-only, 1-4 distinct
    colours from the seven allowed names, ...) is identical to Task 3.
    """
    try:
        colours = parse_command(text, allow_bare_list=True)
    except ValueError as exc:
        return 'error', str(exc)
    return 'ok', colours


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('speech_command')

    defaults = {
        'speech_topic': '/speech_command',
        'engine': 'google',            # google (online) | sphinx (offline)
        'language': 'en-US',
        'microphone_index': -1,        # -1 selects the system default input device
        'ambient_noise_calibration_s': 0.6,
        'phrase_time_limit_s': 6.0,
        'listen_timeout_s': 8.0,
        'debug_text_mode': False,      # true: type a fake transcript instead of using the mic
    }
    for key, value in defaults.items():
        node.declare_parameter(key, value)
    p = {key: node.get_parameter(key).value for key in defaults}

    pub = node.create_publisher(String, p['speech_topic'], 10)

    def handle_transcript(text):
        """Print the required [STT] lines and forward a valid command to Task 3."""
        print('[STT] text=' + text)
        outcome, payload = classify_transcript(text)
        if outcome == 'error':
            print('[STT] error=' + payload)
            return
        print('[STT] colours=' + ', '.join(payload))
        if pub.get_subscription_count() == 0:
            print('Task 3 mission node is not connected to ' + p['speech_topic'] +
                  '. Start task3.launch.py (or task3_demo.launch.py) first.')
            return
        pub.publish(String(data=text))

    if p['debug_text_mode']:
        print('[SPEECH_READY] debug_text_mode=true (typed text stands in for the microphone) '
              'topic=' + p['speech_topic'])
        print('Type an English command as if you had spoken it, e.g. "find red and blue".')
        print("Type 'q' to quit.")
        try:
            while rclpy.ok():
                print('Speech(debug)> ', end='', flush=True)
                line = sys.stdin.readline()
                if not line or line.strip().lower() in ('q', 'quit', 'exit'):
                    break
                text = line.strip()
                if not text:
                    continue
                handle_transcript(text)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return

    try:
        import speech_recognition as sr
    except ImportError as exc:
        print('[STT] error=speech_recognition_library_missing (' + str(exc) + '); '
              'pip install --user SpeechRecognition (and pyaudio), '
              'or apt install python3-speechrecognition python3-pyaudio flac')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    recogniser = sr.Recognizer()
    device_index = None if p['microphone_index'] < 0 else int(p['microphone_index'])
    try:
        microphone = sr.Microphone(device_index=device_index)
    except (OSError, AttributeError) as exc:
        print('[STT] error=microphone_unavailable (' + str(exc) + '); '
              'check PyAudio/PortAudio install and microphone permissions, '
              'or set debug_text_mode:=true to test without a microphone')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    print('[SPEECH_READY] engine=' + p['engine'] + ' language=' + p['language'] +
          ' topic=' + p['speech_topic'])
    print('Press ENTER, then speak one English colour command '
          '(e.g. "find red and blue"). Type q + ENTER to quit instead.')

    try:
        while rclpy.ok():
            print('Speech> ', end='', flush=True)
            line = sys.stdin.readline()
            if not line or line.strip().lower() in ('q', 'quit', 'exit'):
                break
            try:
                with microphone as source:
                    recogniser.adjust_for_ambient_noise(
                        source, duration=p['ambient_noise_calibration_s'])
                    print('[LISTENING] speak now...')
                    audio = recogniser.listen(
                        source, timeout=p['listen_timeout_s'],
                        phrase_time_limit=p['phrase_time_limit_s'])
            except sr.WaitTimeoutError:
                print('[STT] error=no_speech_detected_before_timeout')
                continue
            except OSError as exc:
                print('[STT] error=microphone_read_failed (' + str(exc) + ')')
                continue

            try:
                if p['engine'] == 'sphinx':
                    text = recogniser.recognize_sphinx(audio, language=p['language'])
                else:
                    text = recogniser.recognize_google(audio, language=p['language'])
            except sr.UnknownValueError:
                print('[STT] error=could_not_understand_audio')
                continue
            except sr.RequestError as exc:
                print('[STT] error=recognition_service_unavailable (' + str(exc) + ')')
                continue

            handle_transcript(text)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
