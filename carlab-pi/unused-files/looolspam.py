# main.py
import time
from chat_request import chat_session

prompt = " \
write an explanation as to why I as a student would like to enroll in this course, especially based on my background as a student in the Rabinowitz cancer lab. write a lengthy response in my first person perspective.: \
Life processes depend on over 25 elements whose bioinorganic chemistry is relevant to the environment (biogeochemical cycles), agriculture, and health. CHM 544 surveys the bioinorganic chemistry of the elements. In-depth coverage of key transition metal ions including manganese, iron, copper, and molybdenum focuses on redox roles in anaerobic and aerobic systems and metalloenzymes that activate small molecules and ions, including hydrogen, nitrogen, nitrate, nitric oxide, oxygen, superoxide, and hydrogen peroxide. Appreciation of the structure and reactivity of metalloenzyme systems is critical to understanding life at the molecular level. \
write an explanation as to why I as a student would like to enroll in this course, especially based on my background as a student in the Rabinowitz cancer lab. write a lengthy response in my first person perspective.: \
Life processes depend on over 25 elements whose bioinorganic chemistry is relevant to the environment (biogeochemical cycles), agriculture, and health. CHM 544 surveys the bioinorganic chemistry of the elements. In-depth coverage of key transition metal ions including manganese, iron, copper, and molybdenum focuses on redox roles in anaerobic and aerobic systems and metalloenzymes that activate small molecules and ions, including hydrogen, nitrogen, nitrate, nitric oxide, oxygen, superoxide, and hydrogen peroxide. Appreciation of the structure and reactivity of metalloenzyme systems is critical to understanding life at the molecular level. \
"

def main():
    # Set initial context or model information
    chat_session.set_context("")

    total_tokens = 0

    while True:
        try:
            response = chat_session.send_prompt(prompt+prompt+prompt+prompt+prompt+prompt+prompt+prompt)
            print(response)  # Process the response as needed

            print()
            tokens = response['usage']['total_tokens'] 
            print(tokens)
            total_tokens += tokens
            print(f'total: {total_tokens}\n')

        except Exception as e:
            print("An error occurred:", e)
            
        time.sleep(18)  # Sleep to prevent overloading the CPU, adjust as needed


if __name__ == "__main__":
    # print(context)
    main()


# words = "FBRL"
# words += "X"
# print(words)
# ser.write(bytes(words,'utf-8'))
# for i in range(len(words)):
#     ser.write(bytes(words[i],'utf-8'))
#     time.sleep(0.1)


