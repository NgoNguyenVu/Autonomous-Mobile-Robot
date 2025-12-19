import speech_recognition as sr

print("\n--- DANH SÁCH MIC ĐƯỢC TÌM THẤY ---")
mics = sr.Microphone.list_microphone_names()

for index, name in enumerate(mics):
    print(f"Index: {index}  --- Tên: {name}")

print("---------------------------------------")
