import gc
import datetime
import signal


class Part:
    def __init__(self, name, grade_function,
                 max_points, max_seconds, description):
        if not isinstance(name, str):
            raise Exception("Invalid name: %s" % name)
        if grade_function is not None and not callable(grade_function):
            raise Exception("Invalid gradeFunc: %s" % grade_function)
        if not isinstance(max_points, int):
            raise Exception("Invalid maxPoints: %s" % max_points)
        if max_seconds is not None and not isinstance(max_seconds, int):
            raise Exception("Invalid maxSeconds: %s" % max_seconds)
        self.name = name
        self.grade_function = grade_function
        self.max_points = max_points
        self.max_seconds = max_seconds
        self.description = description

        self.points = 0
        self.seconds = 0
        self.messages = []
        self.failed = False

    def fail(self):
        self.failed = True


class TimeoutFunctionException(Exception):
    pass


class TimeoutFunction:
    def __init__(self, test_function, max_seconds):
        self.max_seconds = max_seconds
        self.function = test_function

    def __call__(self):
        import os
        if os.name == 'nt':
            time_start = datetime.datetime.now()
            result = self.function()
            time_end = datetime.datetime.now()
            time_delta = datetime.timedelta(seconds=self.max_seconds + 1)
            if time_end - time_start > time_delta:
                raise TimeoutFunctionException()
            return result
        else:
            signal.signal(signal.SIGALRM, self.handle_max_seconds)
            signal.alarm(self.max_seconds)
            result = self.function()
            signal.alarm(0)
            return result

    @staticmethod
    def handle_max_seconds(unused_arg1, unused_arg2):
        pass
        # print('TIMEOUT!')
        # raise TimeoutFunctionException()


class Grader:
    def __init__(self):
        self.parts = []
        self.messages = []
        self.current_part = None
        self.fatal_error = False

    def load(self, module_name):
        try:
            return __import__(module_name)
        except Exception as e:
            self.fatal_error = True
            self.fail("Threw exception when importing '%s': %s" %
                      (module_name, e))

    def add_part(
            self, name, grade_function,
            max_points=1, max_seconds=1, description=""):
        """Add a basic test case. The test will be visible to students."""
        part = Part(name, grade_function, max_points, max_seconds, description)
        part.basic = True
        self.parts.append(part)

    def grade(self):
        print('========== START GRADING')
        parts = [part for part in self.parts if part.basic]
        for part in parts:
            if self.fatal_error:
                continue
            print('----- START PART %s: %s' % (part.name, part.description))
            self.current_part = part
            start_time = datetime.datetime.now()
            try:
                TimeoutFunction(part.grade_function, part.max_seconds)()
            except KeyboardInterrupt:
                raise
            except TimeoutFunctionException:
                self.fail('Time limit (%s seconds) exceeded.' %
                          part.max_seconds)
            except MemoryError:
                gc.collect()
                self.fail('Memory limit exceeded.')
            except Exception as e:
                self.fail('Exception thrown: %s -- %s' %
                          (str(type(e)), str(e)))
            except SystemExit:
                self.fail('Unexpected exit.')
            end_time = datetime.datetime.now()
            part.seconds = (end_time - start_time).seconds
            display_points = '%s/%s points' % (part.points, part.max_points)
            print('----- END PART %s [took %s (max allowed %s seconds), %s]'
                  % (part.name, end_time - start_time,
                     part.max_seconds, display_points))
            print()

        total = sum(part.points for part in parts)
        max_total = sum(part.max_points for part in parts)
        print('========== END GRADING [%d/%d points]' % (total, max_total))

    def fail(self, message):
        self.add_message(message)
        if self.current_part:
            self.current_part.points = 0
            self.current_part.fail()
        return False

    def require_is_equal(self, true_answer, predict_answer, tolerance=1e-4):
        if isinstance(true_answer, float) or isinstance(predict_answer, float):
            equal = abs(true_answer - predict_answer) < tolerance
        else:
            equal = true_answer == predict_answer
        if equal:
            if not self.current_part.failed:
                self.current_part.points = self.current_part.max_points
            return True
        else:
            return self.fail("Expected '%s', but got '%s'" %
                             (str(true_answer), str(predict_answer)))

    def add_message(self, message):
        print(message)
        if self.current_part:
            self.current_part.messages.append(message)
        else:
            self.messages.append(message)
