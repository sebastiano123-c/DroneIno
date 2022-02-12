function fnSubmitForm(el){
    console.log(el.name);
    document.getElementById(newId).value = el.value;
}



// document.getElementsByClassName("pid-input").onsubmit(fnSubmitForm(this));

//   document.getElementByClass('pid-input').onsubmit = function() { 
//     console.log(document.getElementById('searchTerm').value);
// };