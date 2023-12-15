"""
UsersInput node that could help with audio user input.

Services
--------
    input [UsersInput]: user input.

Returns
-------
    None

"""
import rclpy
from rclpy.node import Node

import speech_recognition as sr

from trajectory_interfaces.srv import UserInput


# Start Citation [1]
def recognize_speech_from_mic(recognizer, microphone):
    """
    Recognize speech from a microphone.

    Args:
    ----
        recognizer (SpeechRecognizer): An instance of the speech recognition Recognizer class.
        microphone (Microphone): An instance of the speech recognition Microphone class.

    Raises
    ------
        TypeError: If `recognizer` is not an instance of Recogniclearzer.
        TypeError: If `microphone` is not an instance of Microphone.

    Returns
    -------
        dict: A dictionary containing recognition results.
            - 'success' (bool): True if recognition was successful, False otherwise.
            - 'error' (str): An error message if an error occurred during recognition.
            - 'transcription' (str): The recognized speech transcription

    """
    if not isinstance(recognizer, sr.Recognizer):
        error_msg_r = "`recognizer` must be `Recognizer` instance"
        raise TypeError(error_msg_r)

    if not isinstance(microphone, sr.Microphone):
        error_msg_m = "`microphone` must be `Microphone` instance"
        raise TypeError(error_msg_m)

    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    response = {"success": True, "error": None, "transcription": None}

    try:
        response["transcription"] = recognizer.recognize_google(audio)
    except sr.RequestError:
        response["success"] = False
        response["error"] = "API unavailable"
    except sr.UnknownValueError:
        response["error"] = "Unable to recognize speech"

    return response


# End Citation [1]


class UsersInput(Node):
    """
    A UserInput node that could help with audio user input.

    Args:
    ----
        Node (ros node): a node's superclass

    """

    def __init__(self):
        super().__init__("user_input")
        self.srv = self.create_service(UserInput, "input", self.user_input_callback)

    def user_input_callback(self, request, response):
        """
        Process user input.

        Args:
        ----
            request (UserInput): UserInput message representing the user's request.
            response (UserInput.Response): An instance of the UserInput response message.

        Returns
        -------
            UserInput.Response: The processed user input stored in the response message.

        """
        WORDS = ["red", "blue", "yellow", "green", "matt"]
        PROMPT_LIMIT = 5

        recognizer = sr.Recognizer()
        microphone = sr.Microphone()

        final_guess = None

        for j in range(PROMPT_LIMIT):
            self.get_logger().info("Speak!")
            guess = recognize_speech_from_mic(recognizer, microphone)
            if guess["transcription"]:
                if guess["transcription"].lower() not in WORDS:
                    self.get_logger().info(
                        "Sorry, you can't choose that as a target.\n"
                    )
                    continue
                else:
                    break
            if not guess["success"]:
                break
            self.get_logger().info("I didn't catch that. What did you say?\n")
        final_guess = guess["transcription"]
        self.get_logger().info(f"You said: {final_guess}\n")

        response.answer = final_guess
        return response


def entry_point(args=None):
    rclpy.init(args=args)
    user_input = UsersInput()
    rclpy.spin(user_input)
