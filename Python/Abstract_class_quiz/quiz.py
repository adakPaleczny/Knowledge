#Code based on and checked with https://coddy.tech/courses/quiz_generator__python_oop_project
from abc import ABC, abstractmethod

class Question(ABC):
    @abstractmethod
    def print(self):
        pass
    
    @abstractmethod
    def check(self, s):
        pass

class YesNoQuestion(Question):
    def __init__(self, question, answer):
        self.question = question
        self.answer = answer
    
    def print(self):
        print(f"[?] {self.question} (yes/no)")
    
    def check(self, s):
        if s == "yes" and self.answer == True:
            return True
        if s == "no" and self.answer == False:
            return True
        else:
            return False

class OpenQuestion(Question):
    def __init__(self, question, answers):
        self.question = question
        self.answers = answers

    def print(self):
        print(f"[?] {self.question}")
    
    def check(self,s):
        if s in self.answers:
            return True
        else:
            return False

class MultiOptionsQuestion(Question):
    def __init__(self, question, options, answer_index):
        self.question = question
        self.options = options
        self.answer_index = answer_index

    def print(self):
        print(f"[?] {self.question}\n")
        for opt in range (len(self.options)):
            print(f"[{opt+1}] {self.options[opt]}")

    def check(self, s):
        if int(s) == (self.answer_index + 1):
            return True
        else:
            return False

class Quiz:
    def __init__(self, questions):
        self.questions = questions
    
    def start(self):
        ans_tab = []
        for x in self.questions:
            x.print()
            print("\n[+] \n")
        
        for i in range(len(self.questions)):
            answer = input()
            ans_tab.append(self.questions[i].check(answer))

        self.print_results(ans_tab)

    def print_results(self, answers):
        correct = 0
        for i in range(len(answers)):
            if answers[i] == True:
                correct += 1
        print(f"Your score is {correct}/{len(answers)}\n")

        for ans in range(len(answers)):
            if answers[ans] == True:
                print(f"[{ans+1}] Pass")
            else:
                print(f"[{ans+1}] Fail")

if __name__ == "__main__":
    quiz_open = Quiz([OpenQuestion('How are you?', ['Great!', 'Amazing'])
                      , YesNoQuestion('Are you happy?', True)])
    quiz_open.start()

    quiz_list = Quiz([MultiOptionsQuestion("How old are you?", ["5", "46", "51", "different age"],3)])
    # quiz_list.start()
