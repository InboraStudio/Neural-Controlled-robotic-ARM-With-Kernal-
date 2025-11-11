// Mobile Navigation Toggle
const navToggle = document.getElementById('navToggle');
const navLinks = document.querySelector('.nav-links');

if (navToggle) {
    navToggle.addEventListener('click', () => {
        navLinks.classList.toggle('active');
        navToggle.classList.toggle('active');
        
        // Prevent body scroll when menu is open
        if (navLinks.classList.contains('active')) {
            document.body.style.overflow = 'hidden';
        } else {
            document.body.style.overflow = '';
        }
    });
}

// Close mobile menu when clicking on a link
const navItems = document.querySelectorAll('.nav-links a');
navItems.forEach(item => {
    item.addEventListener('click', () => {
        navLinks.classList.remove('active');
        navToggle.classList.remove('active');
        document.body.style.overflow = '';
    });
});

// Close mobile menu when clicking outside
document.addEventListener('click', (e) => {
    if (navLinks && navToggle && 
        !navLinks.contains(e.target) && 
        !navToggle.contains(e.target) && 
        navLinks.classList.contains('active')) {
        navLinks.classList.remove('active');
        navToggle.classList.remove('active');
        document.body.style.overflow = '';
    }
});

// Smooth scroll for anchor links
document.querySelectorAll('a[href^="#"]').forEach(anchor => {
    anchor.addEventListener('click', function (e) {
        e.preventDefault();
        const href = this.getAttribute('href');
        
        // If href is just "#", scroll to top
        if (href === '#') {
            window.scrollTo({
                top: 0,
                behavior: 'smooth'
            });
        } else {
            const target = document.querySelector(href);
            if (target) {
                const offset = 80; // Height of fixed navbar
                const targetPosition = target.offsetTop - offset;
                window.scrollTo({
                    top: targetPosition,
                    behavior: 'smooth'
                });
            }
        }
    });
});

// Code Copy Functionality
function copyCode(button) {
    const codeBlock = button.parentElement.querySelector('pre code');
    if (!codeBlock) {
        console.error('Code block not found');
        return;
    }
    
    const code = codeBlock.textContent.trim();
    
    // Try using the Clipboard API
    if (navigator.clipboard && navigator.clipboard.writeText) {
        navigator.clipboard.writeText(code).then(() => {
            const originalText = button.textContent;
            button.textContent = '✓ Copied!';
            button.style.background = '#10b981';
            
            setTimeout(() => {
                button.textContent = originalText;
                button.style.background = '';
            }, 2000);
        }).catch(err => {
            console.error('Failed to copy:', err);
            // Fallback method
            fallbackCopy(code, button);
        });
    } else {
        // Fallback for older browsers
        fallbackCopy(code, button);
    }
}

// Fallback copy method for older browsers
function fallbackCopy(text, button) {
    const textArea = document.createElement('textarea');
    textArea.value = text;
    textArea.style.position = 'fixed';
    textArea.style.left = '-999999px';
    document.body.appendChild(textArea);
    textArea.select();
    
    try {
        const successful = document.execCommand('copy');
        document.body.removeChild(textArea);
        
        if (successful) {
            const originalText = button.textContent;
            button.textContent = '✓ Copied!';
            button.style.background = '#10b981';
            
            setTimeout(() => {
                button.textContent = originalText;
                button.style.background = '';
            }, 2000);
        } else {
            button.textContent = 'Failed';
            setTimeout(() => {
                button.textContent = 'Copy';
            }, 2000);
        }
    } catch (err) {
        console.error('Fallback copy failed:', err);
        document.body.removeChild(textArea);
        button.textContent = 'Failed';
        setTimeout(() => {
            button.textContent = 'Copy';
        }, 2000);
    }
}

// Navbar scroll effect
let lastScroll = 0;
const navbar = document.querySelector('.navbar');

window.addEventListener('scroll', () => {
    const currentScroll = window.pageYOffset;
    
    if (currentScroll <= 0) {
        navbar.style.boxShadow = '0 1px 2px 0 rgb(0 0 0 / 0.05)';
    } else {
        navbar.style.boxShadow = '0 10px 15px -3px rgb(0 0 0 / 0.1)';
    }
    
    lastScroll = currentScroll;
});

// Intersection Observer for fade-in animations
const observerOptions = {
    threshold: 0.1,
    rootMargin: '0px 0px -50px 0px'
};

const observer = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
        if (entry.isIntersecting) {
            entry.target.style.opacity = '1';
            entry.target.style.transform = 'translateY(0)';
        }
    });
}, observerOptions);

// Observe all cards and sections
document.addEventListener('DOMContentLoaded', () => {
    const animatedElements = document.querySelectorAll('.card, .setup-step, .workflow-card, .arch-card');
    
    animatedElements.forEach(el => {
        el.style.opacity = '0';
        el.style.transform = 'translateY(20px)';
        el.style.transition = 'opacity 0.6s ease-out, transform 0.6s ease-out';
        observer.observe(el);
    });
});

// Add active state to navigation based on scroll position
window.addEventListener('scroll', () => {
    const sections = document.querySelectorAll('section[id]');
    const scrollPosition = window.pageYOffset + 100;
    
    sections.forEach(section => {
        const sectionTop = section.offsetTop;
        const sectionHeight = section.offsetHeight;
        const sectionId = section.getAttribute('id');
        
        if (scrollPosition >= sectionTop && scrollPosition < sectionTop + sectionHeight) {
            document.querySelectorAll('.nav-links a').forEach(link => {
                link.style.color = '';
                if (link.getAttribute('href') === `#${sectionId}`) {
                    link.style.color = '#6366f1';
                }
            });
        }
    });
});

// Syntax highlighting simulation for code blocks
document.addEventListener('DOMContentLoaded', () => {
    const codeBlocks = document.querySelectorAll('.code-block code');
    
    codeBlocks.forEach(block => {
        // Store original text
        const originalText = block.textContent;
        let html = originalText;
        
        // Escape HTML first
        html = html.replace(/</g, '&lt;').replace(/>/g, '&gt;');
        
        // Highlight comments
        html = html.replace(/(#[^\n]*)/g, '<span style="color: #94a3b8;">$1</span>');
        
        // Highlight commands
        html = html.replace(/\b(make|git|cd|sudo|apt-get|brew|pacman|avrdude|clone|push|pull|install|update)\b/g, '<span style="color: #34d399;">$1</span>');
        
        // Highlight flags
        html = html.replace(/(-{1,2}[\w-]+)/g, '<span style="color: #fbbf24;">$1</span>');
        
        block.innerHTML = html;
    });
});

// Add loading animation
window.addEventListener('load', () => {
    document.body.style.opacity = '0';
    setTimeout(() => {
        document.body.style.transition = 'opacity 0.5s';
        document.body.style.opacity = '1';
    }, 100);
});

// Easter egg: Console message
console.log('%c🧠 Neural-Controlled Robotic ARM', 'color: #6366f1; font-size: 20px; font-weight: bold;');
console.log('%cBare-metal EMG signal processing at 9.6 kHz', 'color: #10b981; font-size: 14px;');
console.log('%cInterested in the code? Check out: https://github.com/InboraStudio/Neural-Controlled-robotic-ARM-With-Kernal-', 'color: #64748b; font-size: 12px;');
