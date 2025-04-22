# main.py
# import secrets
import sys

DEVICE_ROLE = 'RC'

def main():
    if DEVICE_ROLE == 'RC':
        module_name = 'rc'
    elif DEVICE_ROLE == 'ROV':
        module_name = 'rov'
    else:
        print('Invalid DEVICE_ROLE specified in py.')
        sys.exit(1)

    try:
        module = __import__(module_name)
        module.run()
    except ImportError:
        print(f'Failed to import module: {module_name}')
        sys.exit(1)

if __name__ == '__main__':
    main()